import rospy


class DataCollector(object):
    """This class handle data collection and retrieval.
    """
    def __init__(self, topic, topic_type, nb_data_points):
        # type: (str, str, int) -> None
        """Initialize the DataCollector class.

        Args:
            topic: Topic name.
            topic_type: Topic type in string, e.g. std_msgs/Float64.
            nb_data_points: Number of latest data points to store.
        """
        if topic_type == "std_msgs/Float64":
            from std_msgs.msg import Float64
            _type = Float64
        elif topic_type == "sensor_msgs/JointState":
            from sensor_msgs.msg import JointState
            _type = JointState
        else:
            raise RuntimeError("Unknown topic type %s" % topic_type)
        self._topic_type = topic_type
        self._sub = rospy.Subscriber(topic, _type, self.callback, queue_size=1)
        self._raw_msgs = []
        self._nb_data_points = nb_data_points
        self._recv_times = []
        self._current_index = 0
        self._total_index = 0
        self.has_new_data = False

    def callback(self, msg):
        """Message receive callback."""
        if len(self._raw_msgs) < self._nb_data_points:
            self._raw_msgs.append(msg)
            self._recv_times.append(rospy.get_time())
        else:
            self._raw_msgs[self._current_index] = msg
            self._recv_times[self._current_index] = rospy.get_time()
            self._current_index = (self._current_index + 1) % self._nb_data_points
            self._total_index += 1
        self.has_new_data = True


