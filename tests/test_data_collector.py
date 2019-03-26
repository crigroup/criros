import pytest
import rospy
import criros.databoard
import yaml
import numpy as np
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import subprocess


def publish_fake_data(topics, datapoints=250, rate=100):
    # type: (dict[str, dict], int, int) -> dict[str, np.ndarray]
    """Publish fake data to given topics."""
    pub_dict = {}
    for topic in topics:
        if topics[topic]["type"] == "std_msgs/Float64":
            pub_dict[topic] = rospy.Publisher(topic, std_msgs.msg.Float64, queue_size=1)
        elif topics[topic]["type"] == "sensor_msgs/JointState":
            pub_dict[topic] = rospy.Publisher(topic, sensor_msgs.msg.JointState, queue_size=1)
        elif topics[topic]['type'] == 'geometry_msgs/Wrench':
            pub_dict[topic] = rospy.Publisher(topic, geometry_msgs.msg.Wrench, queue_size=1)
        elif topics[topic]['type'] == 'geometry_msgs/WrenchStamped':
            pub_dict[topic] = rospy.Publisher(topic, geometry_msgs.msg.WrenchStamped, queue_size=1)
        else:
            raise RuntimeError("Unknown type: %s" % topics[topic]['type'])

    fake_data = {}
    rate = rospy.Rate(rate)
    for i in range(datapoints):
        for topic in topics:
            if topic not in fake_data:
                fake_data[topic] = []
            if topics[topic]["type"] == "std_msgs/Float64":
                msg = std_msgs.msg.Float64()
                msg.data = np.random.randn()
                fake_data[topic].append(msg.data)
            elif topics[topic]["type"] == "sensor_msgs/JointState":
                msg = sensor_msgs.msg.JointState()
                msg.header.stamp = rospy.Time.now()
                msg.position = np.random.randn(6)
                fake_data[topic].append(msg.position)
            elif topics[topic]['type'] == 'geometry_msgs/Wrench':
                msg = geometry_msgs.msg.Wrench()
                d = np.random.randn(6)
                msg.force = geometry_msgs.msg.Vector3(x=d[0], y=d[1], z=d[2])
                msg.torque = geometry_msgs.msg.Vector3(x=d[3], y=d[4], z=d[5])
                fake_data[topic].append(d)
            elif topics[topic]['type'] == 'geometry_msgs/WrenchStamped':
                msg = geometry_msgs.msg.WrenchStamped()
                msg.header.stamp = rospy.Time.now()
                d = np.random.randn(6)
                msg.wrench.force = geometry_msgs.msg.Vector3(x=d[0], y=d[1], z=d[2])
                msg.wrench.torque = geometry_msgs.msg.Vector3(x=d[3], y=d[4], z=d[5])
                fake_data[topic].append(d)
            pub_dict[topic].publish(msg)
        rate.sleep()
    for topic in fake_data:
        fake_data[topic] = np.array(fake_data[topic])
        if topics[topic]["type"] in ["sensor_msgs/JointState"]:
            fake_data[topic] = fake_data[topic].T
    return fake_data


@pytest.fixture(autouse=True, scope='module')
def rospy_setup():
    roscore_process = subprocess.Popen("roscore")
    rospy.init_node("test_databoard")
    yield
    roscore_process.terminate()
    roscore_process.wait()

@pytest.fixture()
def topic_data():
    yaml_string = """
data_board:
  layout: [1, 2]
  number_store_points: 200
  time_window_sec: 120
  topics:
    "/topic_float64":
      type: std_msgs/Float64
      plot_handles:
        - [0, "data"]
    "/topic_joint_states":
      type: sensor_msgs/JointState
      plot_handles:
        - [0, "position[0]"]
        - [1, "position[2]"]
    "/topic_wrenches":
      type: geometry_msgs/Wrench
      plot_handles:
        - [0, "force.x"]
        - [1, "torque.y"]
    "/topic_wrench_stamped":
      type: geometry_msgs/WrenchStamped
      plot_handles:
        - [0, "wrench.force.x"]
        - [1, "wrench.torque.y"]
  axes_configs:
    sharex: True
    yranges:
      - [0, [-40, 40]]
      - [1, [-0.3, 0.3]]"""

    param_dict = yaml.load(yaml_string)
    yield param_dict


def test_data_collector_has_msg(topic_data):
    "DataCollector can listen to messages."
    param = topic_data
    # setup parameters
    rospy.set_param("/", param)
    topics = rospy.get_param('/data_board/topics')
    collector_dict = {}
    for topic in topics:
        collector_dict[topic] = criros.databoard.DataCollector(
            topic, topics[topic]["type"],
            rospy.get_param("/data_board/number_store_points"))
    publish_fake_data(topics, datapoints=250, rate=125)  # publish fake data in 2 seconds.

    # test
    for topic in collector_dict:
        assert collector_dict[topic].has_new_data, "No data received"


def test_retrieve_msg_overflow(topic_data):
    "If more data points are received than total nb of data points, treat overflow correctly."
    param = topic_data
    # setup parameters
    rospy.set_param("/", param)
    topics = rospy.get_param('/data_board/topics')
    collector_dict = {}
    nb_store_pts = rospy.get_param("/data_board/number_store_points")
    for topic in topics:
        collector_dict[topic] = criros.databoard.DataCollector(
            topic, topics[topic]["type"],
            nb_store_pts)

    fake_data = publish_fake_data(topics, datapoints=250, rate=125)  # publish fake data in 2 seconds.

    # test
    for topic in collector_dict:
        assert len(collector_dict[topic]._raw_msgs) == nb_store_pts
        assert len(collector_dict[topic]._recv_times) == nb_store_pts

    # valid data points
    np.testing.assert_allclose(fake_data['/topic_float64'][-200:],
                               collector_dict['/topic_float64'].get_data("data"))

    for i in range(6):
        np.testing.assert_allclose(
            fake_data['/topic_joint_states'][i, -200:],
            collector_dict['/topic_joint_states'].get_data("position[%d]" % i))

    np.testing.assert_allclose(fake_data['/topic_wrenches'][-200:, 0],
                               collector_dict['/topic_wrenches'].get_data("force.x"))
    np.testing.assert_allclose(fake_data['/topic_wrenches'][-200:, 3],
                               collector_dict['/topic_wrenches'].get_data("torque.x"))

    np.testing.assert_allclose(fake_data['/topic_wrench_stamped'][-200:, 0],
                               collector_dict['/topic_wrench_stamped'].get_data("wrench.force.x"))
    np.testing.assert_allclose(fake_data['/topic_wrench_stamped'][-200:, 3],
                               collector_dict['/topic_wrench_stamped'].get_data("wrench.torque.x"))

def test_retrieve_data_underflow(topic_data):
    "If less data points are received than total nb of data points, treat underflow correctly."
    param = topic_data
    # setup parameters
    rospy.set_param("/", param)
    topics = rospy.get_param('/data_board/topics')
    collector_dict = {}
    nb_store_pts = rospy.get_param("/data_board/number_store_points")
    for topic in topics:
        collector_dict[topic] = criros.databoard.DataCollector(
            topic, topics[topic]["type"], nb_store_pts)

    fake_data = publish_fake_data(topics, datapoints=100, rate=125)  # publish fake data in 2 seconds.

    # test
    for topic in collector_dict:
        assert len(collector_dict[topic]._raw_msgs) == 100
        assert len(collector_dict[topic]._recv_times) == 100

    # valid data points
    np.testing.assert_allclose(fake_data['/topic_float64'][-50:],
                               collector_dict['/topic_float64'].get_data("data")[-50:])

    for i in range(6):
        np.testing.assert_allclose(
            fake_data['/topic_joint_states'][i, -50:],
            collector_dict['/topic_joint_states'].get_data("position[%d]" % i)[-50:])

    np.testing.assert_allclose(fake_data['/topic_wrenches'][-200:, 0],
                               collector_dict['/topic_wrenches'].get_data("force.x"))
    np.testing.assert_allclose(fake_data['/topic_wrenches'][-200:, 3],
                               collector_dict['/topic_wrenches'].get_data("torque.x"))


@pytest.mark.parametrize("datapoints", [100, 200, 400])
def test_data_collector_monotonic_time(topic_data, datapoints):
    "Collected data should have monotonically increasing time."
    param = topic_data
    # setup parameters
    rospy.set_param("/", param)
    topics = rospy.get_param('/data_board/topics')
    collector_dict = {}
    nb_store_pts = rospy.get_param("/data_board/number_store_points")
    for topic in topics:
        collector_dict[topic] = criros.databoard.DataCollector(
            topic, topics[topic]["type"], nb_store_pts)

    publish_fake_data(topics, datapoints=datapoints, rate=125)  # publish fake data in 2 seconds.

    # monotonically increasing time with low variance
    for topic in topics:
        t_vec = collector_dict[topic].get_time()
        td_vec = np.diff(t_vec)
        assert np.all(td_vec > 0)
        assert np.max(td_vec) < 0.012
        assert np.min(td_vec) > 0.003
        np.testing.assert_almost_equal(np.mean(td_vec), 0.008, decimal=1)

