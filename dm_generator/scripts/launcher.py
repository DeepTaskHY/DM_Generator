#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import rospy
from dtroslib.dialogflow import DialogflowClient
from dtroslib.helpers import get_module_configuration, get_key_path
from ros import DMNode


def main():
    rospy.init_node('DM_node')

    scenario_name = rospy.get_param('~scenario_name')
    configuration = get_module_configuration('dm_generator', scenario_name)
    key_path = get_key_path('dm_generator', configuration['authorization']['key'])

    dialogflow_client = DialogflowClient(project_id=configuration['project-id'],
                                         session_id=configuration['session-id'],
                                         key_path=key_path,
                                         language_code=configuration['language-code'])

    node = DMNode(node_name='DM_node',
                  scenario_name=scenario_name,
                  language_code=configuration['language-code'],
                  dialogflow_client=dialogflow_client)

    rospy.loginfo(f'Start DM ({scenario_name})')

    node.spin()


if __name__ == '__main__':
    main()
