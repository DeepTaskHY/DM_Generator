from abc import *
from typing import Callable, Tuple

import time
import json
import random
import rospy
from std_msgs.msg import String
from dialogflow import DialogflowClient
from scenarios import ScenarioParser, Scenario
from helpers import timestamp


# Base Node of ROS
class NodeBase:
    def __init__(self, node_name: str):
        self.node_name = node_name
        self.publishers = {}
        self.subscribers = {}

        # ROS module initialization
        rospy.init_node(self.node_name)

    def add_publisher(self,
                      topic_name: str,
                      *args, **kwargs):

        # Initialize publisher
        publisher = rospy.Publisher(topic_name, *args, **kwargs)

        # Delete exist publisher
        if topic_name in self.publishers:
            del self.publishers[topic_name]

        # Add publisher to list
        self.publishers[topic_name] = publisher

        return publisher

    def add_subscriber(self,
                       topic_name: str,
                       *args, **kwargs):

        # Initialize publisher
        subscriber = rospy.Subscriber(topic_name, *args, **kwargs)

        # Delete exist publisher
        if topic_name in self.subscribers:
            del self.subscribers[topic_name]

        # Add publisher to list
        self.subscribers[topic_name] = subscriber

        return subscriber

    def publish(self,
                topic_name: str,
                *args, **kwargs):

        publisher = self.publishers[topic_name]
        return publisher.publish(*args, **kwargs)

    @classmethod
    def spin(cls):
        rospy.spin()


# Node of DeepTask
class DTNode(NodeBase, metaclass=ABCMeta):
    def __init__(self,
                 publish_message: str,
                 subscribe_message: str,
                 queue_size: int = 10,
                 *args, **kwargs):

        super(DTNode, self).__init__(*args, **kwargs)
        self.source_name = None
        self.publish_message = publish_message
        self.subscribe_message = subscribe_message
        self.add_publisher(publish_message, String, queue_size=queue_size)
        self.add_subscriber(subscribe_message, String, self.subscribe)

    def subscribe(self, message):
        received_message = json.loads(message.data)
        rospy.loginfo(f'Received message: {received_message}')

        # Check receiver
        if self.source_name and self.source_name not in received_message['header']['target']:
            return

        header = received_message['header']
        content_names = header['content']

        if isinstance(content_names, str):
            content_names = [content_names]

        for content_name in content_names:
            content = received_message[content_name]
            generated_content_name, generated_content, target_list = self.generate_content(content_name, content)

            if generated_content_name:
                generated_message = self.generate_message(generated_content_name, target_list, **generated_content)
                self.publish(self.publish_message, json.dumps(generated_message, ensure_ascii=False))
                rospy.loginfo(f'Published message: {generated_message}')

    # Generate DeepTask ROS module output message
    def generate_message(self,
                         content_name: str,
                         target_list: list,
                         **kwargs) -> dict:

        message = {
            'header': {
                'content': content_name,
                'source': self.source_name,
                'target': target_list,
                'timestamp': timestamp()
            },
            content_name: kwargs
        }

        return message

    # Generate specific ROS module output content
    @abstractmethod
    def generate_content(self,
                         content_name: str,
                         content: dict) -> Tuple[str, list, dict]:

        pass


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
    def scenario(self):
        if not self.__scenario:
            scenario_parser = ScenarioParser(self.scenario_name)
            self.__scenario = scenario_parser.get_scenario()

        return self.__scenario

    def generate_content(self,
                         content_name: str,
                         content: dict) -> Tuple[str, list, dict]:

        if content_name == 'dialog_generation':
            # Add DialogFlow result
            if 'human_speech' in content:
                human_speech = content['human_speech']
                dialogflow_result = self.dialogflow_client.detect_intent_text(human_speech)
                content.update({'dialogflow': dialogflow_result})

            # Instantiate Intent
            intent_name = content['intent']
            intent = self.scenario.get_intent(intent_name)
            intent.set_parameter_content(content)

            # Get correct dialog
            dialog = random.choice(intent.correct_dialogs)

            # Publish message
            target_list = [
                'planning'
            ]

            generated_content = {
                'id': content['id'],
                'dialog': dialog.value[self.language_code],
                'result': 'completion'
            }

            return content_name, generated_content, target_list

        return None, None, None
