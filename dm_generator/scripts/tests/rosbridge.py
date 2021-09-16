from flask_socketio import Namespace
from roslibpy import Ros, Topic, Message


class RosBridgeNamespace(Namespace):
    __ros: Ros = None
    __ros_host: str = None
    __ros_port: int = None

    def __init__(self, host: str, port: int, *args, **kwargs):
        super(RosBridgeNamespace, self).__init__(*args, **kwargs)
        self.__ros_host = host
        self.__ros_port = port
        self.register_subscribe()

    @property
    def ros(self):
        if not self.__ros:
            self.__ros = Ros(host=self.__ros_host, port=self.__ros_port)
            self.__ros.run()

        return self.__ros

    def on_publish(self, data):
        publisher = Topic(self.ros, '/taskExecution', 'std_msgs/String')
        publisher.publish(Message(data))

    def register_subscribe(self):
        subscriber = Topic(self.ros, '/taskCompletion', 'std_msgs/String')
        subscriber.subscribe(self.subscribe)

    def subscribe(self, data):
        self.emit('subscribe', data)
