import rospy
import numpy as np
from rospy_message_converter import message_converter
import regex as re
import logging

logger = logging.getLogger("criros.databoard")


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
        elif topic_type == 'geometry_msgs/Wrench':
            from geometry_msgs.msg import Wrench
            _type = Wrench
        elif topic_type == 'geometry_msgs/WrenchStamped':
            from geometry_msgs.msg import WrenchStamped
            _type = WrenchStamped
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

    def get_data(self, select_string=""):
        # type: (str) -> (np.ndarray, np.ndarray)
        """Get data with select_string.

        Args:
            select_string: Selection string.

        Returns:
            A tuple of times and data points.

        """
        _total_index_local = int(self._total_index)
        data = np.zeros(self._nb_data_points)
        times = np.zeros(self._nb_data_points)
        extract_func = DataCollector.construct_extract_func(self._topic_type, select_string)
        if _total_index_local < self._nb_data_points:
            for i in range(_total_index_local):
                data[i] = extract_func(self._raw_msgs[i])
            times = self._recv_times[:_total_index_local]
            data = data[:_total_index_local]
        else:
            for i in range(self._nb_data_points):
                data[self._nb_data_points - i - 1] = extract_func(self._raw_msgs[self._current_index - i - 1])
                times[self._nb_data_points - i - 1] = self._recv_times[self._current_index - i - 1]
        # logger.debug("data: %s", data)
        return times, data

    def get_time(self):
        """Get time associated with data points."""
        times = np.array(self._recv_times)
        if self._total_index < self._nb_data_points:
            pass
        else:
            for i in range(self._nb_data_points):
                times[self._nb_data_points - i - 1] = self._recv_times[self._current_index - i - 1]
        # logger.debug("time: %s", times)
        return times

    @staticmethod
    def construct_extract_func(topic_type, select_string):
        """Return a function to extract data from a message."""
        if select_string == "":
            return lambda msg: np.nan
        if topic_type == "std_msgs/Float64":
            func = lambda msg: msg.data
        elif topic_type in ["sensor_msgs/JointState", 'geometry_msgs/Wrench', 'geometry_msgs/WrenchStamped']:
            # input: `some_file[x]`; output: msg.some_field[x]
            match = re.match('([a-z]*)(\.[a-z]*)*(\[([0-9])\])?', select_string)
            if match is None:
                raise RuntimeError("Fail to capture regex for string: %s" % select_string)
            field_base = match.group(1)
            fields_other = [s[1:] for s in match.captures(2)]
            if len(match.captures(4)) != 0:
                index = int(match.group(4))
            else:
                index = None
            def func(msg):
                data = message_converter.convert_ros_message_to_dictionary(msg)[field_base]
                for field in fields_other:
                    data = data[field]
                if index is not None:
                    data = data[index]
                return data
        else:
            raise RuntimeError("Unknown topic type: [%s]" % topic_type)
        return func



