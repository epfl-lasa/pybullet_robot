import rospy


class ROSInterface(object):
    def __init__(self):
        rospy.init_node('simulation_interface', anonymous=False, disable_signals=False)
        self.subscribers = {}

    def __del__(self):
        rospy.signal_shutdown("SIGINT")

    @staticmethod
    def is_connected():
        return ~rospy.is_shutdown()

    @staticmethod
    def get_time():
        return rospy.Time.now()

    @staticmethod
    def add_publisher(topic_name, msg_type, queue_size=1):
        return rospy.Publisher(topic_name, msg_type, queue_size=queue_size)

    def add_subscriber_callback(self, topic_name, msg_type, callback_functions):
        callback_functions = [callback_functions] if type(callback_functions) is not list else callback_functions
        if topic_name not in self.subscribers.keys():
            rospy.Subscriber(topic_name, msg_type, self.callback, topic_name)
            self.subscribers[topic_name] = {"type": msg_type, "callbacks": callback_functions}
        else:
            if self.subscribers[topic_name]["type"] is msg_type:
                self.subscribers[topic_name]["callbacks"].extend(callback_functions)
            else:
                raise ValueError(
                    "There is already a subscriber connected to the desired topic with a different message type (%s) !" %
                    self.subscribers[topic_name]["type"])

    def callback(self, data, topic_name):
        for cb in self.subscribers[topic_name]["callbacks"]:
            cb(data)
