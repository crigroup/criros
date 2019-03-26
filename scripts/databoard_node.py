#!/usr/bin/env python
"""A convenient program to view data from published topics.

Users interact with this program by writing a ROS configuration file,
loaded to ROS parameter server.

The script can work with any kind of numerical data from ROS topics.

"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
import logging
import numpy as np
from criros.databoard import DataCollector

logger = logging.getLogger('criros.databoard')


if __name__ == '__main__':
    nh = rospy.init_node("data_board")
    layout = rospy.get_param("/data_board/layout")
    logger.info("layout: {:}".format(layout))

    time_window_sec = rospy.get_param("/data_board/time_window_sec")
    number_store_points = rospy.get_param("/data_board/number_store_points")
    topics = rospy.get_param("/data_board/topics")
    yranges = rospy.get_param("/data_board/axes_configs/yranges")
    grid_on = rospy.get_param("/data_board/axes_configs/grid_on")
    sharex = rospy.get_param("/data_board/axes_configs/sharex")

    # preprocess topic name (change \ to /)
    old_keys = topics.keys()
    for key in old_keys:
        new_key = key.replace('^', "/")
        topics[new_key] = topics[key]
        del topics[key]
    logger.debug("topics: \n%s", topics)

    # subscribe to topic and do stuff
    data_collector_dict = {}
    for topic in topics:
        data_collector_dict[topic] = DataCollector(topic, topics[topic]["type"], number_store_points)

    # Create figure for plotting
    fig, axs = plt.subplots(layout[0], layout[1], sharex=sharex)
    ys = []

    # Create a blank line. We will update the line in animate
    lines = []
    for topic in topics:
        topics[topic]['lines'] = []
        for ax_index, select_cmd in topics[topic]['plot_handles']:
            line, = axs[ax_index].plot([0, 1], [1, 1], '-', label="%s,%s" % (topic, select_cmd))
            topics[topic]["lines"].append(line)
            lines.append(line)
            axs[ax_index].legend()

    for axes_index, (ymin, ymax) in yranges:
        axs[axes_index].set_ylim(ymin, ymax)

    for axes_index in grid_on:
        axs[axes_index].grid()

    t_zero = rospy.get_time()

    # This function is called periodically from FuncAnimation
    xleft = 0
    def animate(i, xleft):
        for topic in topics:
            for i, (ax_index, select_cmd) in enumerate(topics[topic]['plot_handles']):
                xdata = data_collector_dict[topic].get_time() - t_zero
                ydata = data_collector_dict[topic].get_data(select_cmd)

                topics[topic]["lines"][i].set_xdata(xdata)
                topics[topic]["lines"][i].set_ydata(ydata)
                if len(xdata) == 0:
                    continue
                try:
                    xleft = max(xdata[-1] - time_window_sec * 0.75, xleft)
                except IndexError as e:
                    logger.warn("Exception msg: %s", e.message)
                    logger.warn("xdata: %s. topic: %s. select cmd: %s", xdata, topic, select_cmd)
        axs[ax_index].set_xlim(xleft, xleft + time_window_sec)
        return lines

    # Set up plot to call animate() function periodically
    ani = animation.FuncAnimation(fig,
                                  animate,
                                  interval=250, blit=False, fargs=(xleft,))
    plt.show()
