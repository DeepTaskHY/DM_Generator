from abc import *
from typing import Callable
import time
import json
import rospy
from std_msgs.msg import Message, String
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

        # Check receiver
        if self.source_name and self.source_name not in received_message['header']['target']:
            return

        header = received_message['header']
        content = received_message[header['content']]
        content_name, targets, generated_content = self.generate_content(header, content)
        generated_message = self.generate_message(content_name, targets, **generated_content)
        self.publish(self.publish_message, json.dumps(generated_message, ensure_ascii=False))

    # Generate DeepTask ROS module output message
    def generate_message(self,
                         content: str,
                         target: list,
                         **kwargs):

        message = {
            'header': {
                'content': content,
                'source': self.source_name,
                'target': target,
                'timestamp': timestamp()
            },
            content: kwargs
        }

        return message

    # Generate specific ROS module output content
    @abstractmethod
    def generate_content(self,
                         header: dict,
                         content: dict) -> tuple(str, list, dict):
        pass


# Node of Dialog Manager
class DMNode(DTNode):
    def __init__(self, *args, **kwargs):
        super(DMNode, self).__init__(*args, **kwargs)
        self.source_name = 'dialog'
