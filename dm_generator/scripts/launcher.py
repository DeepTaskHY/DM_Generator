#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import rospy
from dtroslib.dialogflow import DialogflowClient
from dtroslib.helpers import get_module_configuration, get_key_path
from ros import DMNode
from helpers import get_model_path
from conversation import load_conversation_model


def main():
    # Load node
    node_name = 'DM_node'
    rospy.init_node(node_name)
    scenario_name = rospy.get_param('~scenario_name')

    # Load configuration
    configuration = get_module_configuration('dm_generator', scenario_name)

    # Set configuration
    project_id = configuration['project-id']
    key_path = get_key_path('dm_generator', configuration['authorization']['key'])
    language_code = configuration['language-code']

    # Initialize nodes
    dialogflow_client = DialogflowClient(project_id=project_id,
                                         key_path=key_path,
                                         language_code=language_code)

    # Load conversation model
    model_path = get_model_path('conversation/saved_model')
    conversation_model = load_conversation_model(model_path)

    node = DMNode(node_name=node_name,
                  scenario_name=scenario_name,
                  language_code=language_code,
                  dialogflow_client=dialogflow_client,
                  conversation_model=conversation_model)

    rospy.loginfo(f'Start DM ({scenario_name})')

    node.spin()


if __name__ == '__main__':
    main()
