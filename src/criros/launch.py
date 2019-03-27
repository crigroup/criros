import subprocess
import roslaunch
import logging

logger = logging.getLogger('criros.launch')


def try_start_master():
    """Try to start a rosmaster.

    If a rosmaster is already running, do nothing.

    Returns:
        rosmaster process.

    """
    if not roslaunch.Master().is_running():
        logger.info("ROS master is not running. Starting roscore.")
        master_proc = subprocess.Popen('roscore')
    else:
        logger.info('ROS master is running. Do nothing.')
        master_proc = None
    return master_proc

