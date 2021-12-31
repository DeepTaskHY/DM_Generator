#!/usr/bin/python3
# -*- coding: utf-8 -*-
from dtroslib.dialogflow import DialogflowClient
from dtroslib.ros import DTNode
from eval_conversation import predict
from scenarios import ScenarioParser, Scenario
from tensorflow.keras.models import Model
from typing import List, Dict, Tuple


# Node of Dialog Manager
class DMNode(DTNode):
    __scenario: Scenario = None

    def __init__(self,
                 scenario_name: str,
                 language_code: str,
                 dialogflow_client: DialogflowClient,
                 conversation_model: Model,
                 *args, **kwargs):

        super(DMNode, self).__init__(publish_message='/taskCompletion',
                                     subscribe_message='/taskExecution',
                                     *args, **kwargs)

        self.source_name = 'dialog'
        self.scenario_name = scenario_name
        self.dialogflow_client = dialogflow_client
        self.conversation_model = conversation_model
        self.language_code = language_code

    @property
    def scenario(self) -> Scenario:
        if not self.__scenario:
            scenario_parser = ScenarioParser(self.scenario_name)
            self.__scenario = scenario_parser.get_scenario()

        return self.__scenario

    def dialog_generation(self, content: dict) -> str:
        intent_name = content.get('intent')
        intent = self.scenario.get_intent(intent_name, self.language_code, content)

        if not intent.exist:
            return None

        # Add DialogFlow and Conversation result
        if content.get('human_speech'):
            human_speech = content.get('human_speech')

            if intent.event.result:
                self.dialogflow_client.trigger_intent_event(intent.event.name)

            dialogflow_result = self.dialogflow_client.detect_intent_text(human_speech)
            content.update(dialogflow=dialogflow_result)

            conversation_result = predict(self.conversation_model, human_speech)
            content.update(conversation=conversation_result)

            # Update content
            intent.set_parameter_content(content)

        # Generate dialog
        dialog = intent.get_correct_dialog()

        if not dialog and content.get('dialogflow'):
            dialogflow_result = content.get('dialogflow')
            dialog = dialogflow_result.query_result.fulfillment_text

        return dialog

    def generate_contents(self,
                          source: str,
                          contents: Dict[str, dict]) -> Tuple[List[str], Dict[str, dict]]:

        targets = []
        generated_contents = {}

        for content_name, content in contents.items():
            if content_name == 'dialog_generation':
                dialog = self.dialog_generation(content)

                if not dialog:
                    continue

                targets.append(source)

                generated_contents.update(dialog_generation={
                    'dialog': dialog
                })

        # Remove duplicate targets
        targets = list(set(targets))

        return targets, generated_contents
