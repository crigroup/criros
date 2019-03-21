import pytest
import rospy
import criros.databoard
import yaml


@pytest.fixture(autouse=True)
def rospy_setup():
    rospy.init_node("test_databoard")


@pytest.fixture
def topic_data():
    yaml_string = """
data_board:
  layout: [1, 2]
  number_store_points: 12500
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

  axes_configs:
    sharex: True
    yranges:
      - [0, [-40, 40]]
      - [1, [-0.3, 0.3]]"""
    param_dict = yaml.load(yaml_string)

    fake_data = {
        "/topic_float64": np.random.randn(1000),
        "/topic_joint_states": np.random.randn(6, 1000)
    }
    yield param_dict, fake_data


def test_data_collector_has_msg(topic_data):
    "DataCollector can listen to messages."
    param, fake_data = topic_data
    # setup parameters
    rospy.set_param("/", param)
    topics = rospy.get_param('/data_board/topics')
    collector_dict = {}
    for topic in topics:
        collector_dict[topic] = criros.databoard.DataCollector(
            topic, topics[topic]["type"],
            rospy.get_param("/data_board/number_store_points"))


    publish_fake_data(datapoint=250, rate=125)  # publish fake data in 2 seconds.

    # test
    for topic in collector_dict:
        assert collector_dict[topic].has_new_data, "No data received"


def test_data_collector_retreive_msg(topic_data):
    "Retrieve messages correctly."
    param, fake_data = topic_data
    # setup parameters
    rospy.set_param("/", param)
    topics = rospy.get_param('/data_board/topics')
    collector_dict = {}
    for topic in topics:
        collector_dict[topic] = criros.databoard.DataCollector(
            topic, topics[topic]["type"],
            rospy.get_param("/data_board/number_store_points"))

    publish_fake_data(datapoints=250, rate=125, data=fake_data)  # publish fake data in 2 seconds.

    # test
    for topic in collector_dict:
        assert(collector_dict[topic].data) == 250

    np.testing.assert_allclose(fake_data['/topic_float64'],
                               collector_dict['/topic_float64'].get_data("data"))

    for i in range(6):
        np.testing.assert_allclose(
            fake_data['/topic_joint_states'][0],
            collector_dict['/topic_joint_states'].get_data("position[0]"))
