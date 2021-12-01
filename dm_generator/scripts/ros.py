#!/usr/bin/python3
# -*- coding: utf-8 -*-
from dtroslib.dialogflow import DialogflowClient
from dtroslib.ros import DTNode
from scenarios import ScenarioParser, Scenario
from typing import List, Dict, Tuple


# Node of Dialog Manager
class DMNode(DTNode):
    __scenario: Scenario = None

    def __init__(self,
                 scenario_name: str,
                 language_code: str,
                 dialogflow_client: DialogflowClient,
                 *args, **kwargs):

        super(DMNode, self).__init__(publish_message='/taskCompletion',
                                     subscribe_message='/taskExecution',
                                     *args, **kwargs)

        self.source_name = 'dialog'
        self.scenario_name = scenario_name
        self.dialogflow_client = dialogflow_client
        self.language_code = language_code

    @property
    def scenario(self) -> Scenario:
        if not self.__scenario:
            scenario_parser = ScenarioParser(self.scenario_name)
            self.__scenario = scenario_parser.get_scenario()

        return self.__scenario

    def dialog_generation(self, content: dict) -> str:
        # Instantiate Intent
        intent_name = content['intent']
        intent = self.scenario.get_intent(intent_name, content)

        # Handling intent exceptions without scenarios
        if not intent.exist:
            return None

        # Add DialogFlow result
        if 'human_speech' in content:
            # Trigger event
            if intent.event.result:
                self.dialogflow_client.trigger_intent_event(intent.event.name)

            # Detect text
            human_speech = content['human_speech']
            dialogflow_result = self.dialogflow_client.detect_intent_text(human_speech)
            content.update({'dialogflow': dialogflow_result})

            # Update dialogflow parameters
            intent.set_parameter_content(content)

        # Generate dialogs
        intent.set_language_code(self.language_code)
        dialog = intent.get_correct_dialog()

        # Out of scenario exception
        if not dialog and 'dialogflow' in content:
            dialogflow_result = content['dialogflow']
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
