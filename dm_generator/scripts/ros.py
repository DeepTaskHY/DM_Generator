#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random
from dtroslib.dialogflow import DialogflowClient
from dtroslib.ros import DTNode
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

    def generate_content(self,
                         source: str,
                         content_names: list,
                         contents: Dict[str, dict]) -> Tuple[list, list, Dict[str, dict]]:

        for content_name in content_names:
            content = contents[content_name]

            if content_name == 'dialog_generation':
                # Instantiate Intent
                intent_name = content['intent']
                intent = self.scenario.get_intent(intent_name)

                # Handling intent exceptions without scenarios
                if not intent.exist:
                    continue

                # Add DialogFlow result
                if 'human_speech' in content:
                    human_speech = content['human_speech']
                    dialogflow_result = self.dialogflow_client.detect_intent_text(human_speech)
                    content.update({'dialogflow': dialogflow_result})

                intent.set_parameter_content(content)

                # Generate dialog
                try:
                    dialogs = intent.get_correct_dialogs()
                    selected_dialog = random.choice(dialogs)
                    generated_dialog = selected_dialog.value[self.language_code]

                # Out of scenario exception
                except IndexError:
                    if 'dialogflow' in content:
                        generated_dialog = content['dialogflow'].query_result.fulfillment_text
                    else:
                        generated_dialog = None

                # Publish message
                targets = source

                if isinstance(targets, str):
                    targets = [targets]

                content_names = ['dialog_generation']

                generated_contents = {
                    'dialog_generation': {
                        'dialog': generated_dialog,
                        'result': 'completion'
                    }
                }

                return targets, content_names, generated_contents

        return None, None, None
