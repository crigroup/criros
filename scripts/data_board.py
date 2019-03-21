#!/usr/bin/env python
"""A convenient program to view data from published topics.

Users interact with this program by writing a ROS configuration file,
loaded to ROS parameter server.

The script can work with any kind of numerical data from ROS topics.

"""
import rospy

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from std_msgs.msg import Float64MultiArray


class DataCollector(object):
    """ Listen to identification data and fitting.
    """
    def __init__(self, topic_name, number_data_points):
        self.data_subscriber = rospy.Subscriber(
            topic_name, Float64MultiArray, self.callback)

        self.data = np.zeros((number_data_points, 7))
        self.time = np.zeros(number_data_points)
        self.number_data_points = number_data_points
        self.current_index = 0
        self.total_index = 0
        self.has_new_data = False

    def callback(self, msg):
        """

        :type msg: Float64MultiArray
        """
        self.data[self.current_index, 0:len(msg.data)] = msg.data
        self.current_index = (self.current_index + 1) % self.number_data_points
        self.total_index += 1
        self.has_new_data = True


if __name__ == '__main__':
    nh = rospy.init_node("data_board")
    layout = rospy.get_param("/data_board/layout")
    rospy.loginfo("layout: {:}".format(layout))

    time_window_sec = rospy.get_param("/data_board/time_window_sec")

    number_store_points = rospy.get_param("/data_board/number_store_points")
    handles = rospy.get_param("/data_board/handles")
    yranges = rospy.get_param("/data_board/axes_configs/yranges")
    try:
        grid_on = rospy.get_param("/data_board/axes_configs/grid_on")
    except KeyError:
        grid_on = []

    try:
        sharex = rospy.get_param("/data_board/axes_configs/sharex")
    except KeyError:
        sharex = True

    # subscribe to topic and do stuff
    data_collector_dict = {}
    for axes_index, topic_name, indies in handles:
        if topic_name in data_collector_dict:
            continue
        else:
            data_collector_dict[topic_name] = DataCollector(topic_name, number_store_points)

    # Create figure for plotting
    fig, axs = plt.subplots(layout[0], layout[1], sharex=sharex)
    ys = []

    # Create a blank line. We will update the line in animate
    lines = []
    for axes_index, topic_name, indies in handles:
        label = topic_name.split("/")[-1] + "[{:}]".format(indies)
        line, = axs[axes_index].plot([0, 1], [1, 1], '-', label=label)
        lines.append(line)
        axs[axes_index].legend()

    for axes_index, (ymin, ymax) in yranges:
        axs[axes_index].set_ylim(ymin, ymax)

    for axes_index in grid_on:
        axs[axes_index].grid()

    handles_latest_total_indices = [-1] * len(handles)

    # This function is called periodically from FuncAnimation
    def animate(i):

        # handle each item in handles list
        for handle_idx, (axes_index, topic_name, indies) in enumerate(handles):
            coll = data_collector_dict[topic_name]

            if handles_latest_total_indices[handle_idx] == coll.total_index:
                continue
            else:
                handles_latest_total_indices[handle_idx] = coll.total_index

            xdata = np.array(
                range(coll.total_index - number_store_points,
                      coll.total_index)) * 0.008
            
            ydata = (coll.data[coll.current_index:, indies].tolist()
                     + coll.data[0:coll.current_index, indies].tolist())

            lines[handle_idx].set_xdata(xdata)
            lines[handle_idx].set_ydata(ydata)

            # current time instance iw 3 quarter accross the screen
            if coll.total_index * 0.008 < time_window_sec * 0.75:
                lower_index = 0
            else:
                lower_index = coll.total_index * 0.008 - time_window_sec * 0.75
            axs[axes_index].set_xlim(lower_index, lower_index + time_window_sec)

        return lines

    # Set up plot to call animate() function periodically
    ani = animation.FuncAnimation(fig,
                                  animate,
                                  interval=250, blit=False)
    plt.show()
