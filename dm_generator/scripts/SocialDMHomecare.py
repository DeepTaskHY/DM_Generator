#!/usr/bin/env python
# -*- coding: utf-8 -*-
from ros import DMNode
from dialogflow import DialogflowClient
from helpers import get_module_configuration


MODULE_NAME = 'homecare'
configuration = get_module_configuration(MODULE_NAME)


def main():
    dialogflow_client = DialogflowClient(project_id=configuration['project-id'],
                                         session_id=configuration['session-id'],
                                         key_file=configuration['authorization']['key'],
                                         language_code=configuration['language-code'])

    node = DMNode(node_name=configuration['node-name'],
                  scenario_name=configuration['scenario-name'],
                  language_code=configuration['language-code'],
                  dialogflow_client=dialogflow_client)

    node.spin()


if __name__ == '__main__':
    main()
