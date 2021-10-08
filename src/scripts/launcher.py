#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from ros import DMNode
from dialogflow import DialogflowClient
from dtroslib.helpers import get_module_configuration, get_key_path
import rospy


module_name = os.environ['DM_NODE']
configuration = get_module_configuration('dm_generator', module_name)
key_path = get_key_path('dm_generator', configuration['authorization']['key'])


def main():
    dialogflow_client = DialogflowClient(project_id=configuration['project-id'],
                                         session_id=configuration['session-id'],
                                         key_path=key_path,
                                         language_code=configuration['language-code'])

    node = DMNode(node_name=configuration['node-name'],
                  scenario_name=configuration['scenario-name'],
                  language_code=configuration['language-code'],
                  dialogflow_client=dialogflow_client)

    rospy.loginfo(f'Start DM ({module_name})')

    node.spin()


if __name__ == '__main__':
    main()
