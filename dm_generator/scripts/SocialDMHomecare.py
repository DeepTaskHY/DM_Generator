#!/usr/bin/env python
# -*- coding: utf-8 -*-
from ros import DMNode
from dialogflow import DialogflowClient
from helpers import get_module_configuration


MODULE_NAME = 'homecare'
configuration = get_module_configuration(MODULE_NAME)


def main():
    language_code = 'ko'

    dialogflow_client = DialogflowClient(project_id=configuration['project-id'],
                                         session_id=configuration['session-id'],
                                         key_file=configuration['authorization']['key'],
                                         language_code=language_code)

    node = DMNode(node_name='DM_homecare_node',
                  scenario_name=MODULE_NAME,
                  dialogflow_client=dialogflow_client,
                  language_code=language_code)

    node.spin()


if __name__ == '__main__':
    main()
