#!/usr/bin/python3
# -*- coding: utf-8 -*-
from dtroslib.dialogflow import DialogflowClient
from dtroslib.ros import DTNode
from google.cloud import dialogflow_v2 as dialogflow
from scenarios import ScenarioParser, Scenario
from typing import Dict, Tuple


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
        try:
            dialogs = intent.get_correct_dialogs()
            generated_dialogs = [dialog.selected_value(self.language_code) for dialog in dialogs]

        # Out of scenario exception
        except IndexError:
            if 'dialogflow' in content:
                dialogflow_result = content['dialogflow']
                generated_dialogs = [dialogflow_result.query_result.fulfillment_text]
            else:
                generated_dialogs = []

        # Dialog result
        dialog = ' '.join(generated_dialogs)

        return dialog

    def generate_content(self,
                         source: str,
                         content_names: list,
                         contents: Dict[str, dict]) -> Tuple[list, list, Dict[str, dict]]:

        for content_name in content_names:
            targets = source
            content = contents[content_name]
            content_names = [content_name]
            generated_contents = {}

            if isinstance(targets, str):
                targets = [targets]

            if content_name == 'dialog_generation':
                dialog = self.dialog_generation(content)

                if not dialog:
                    continue

                generated_contents = {
                    'dialog_generation': {
                        'dialog': dialog,
                        'result': 'completion'
                    }
                }

            if generated_contents:
                return targets, content_names, generated_contents

        return None, None, None
