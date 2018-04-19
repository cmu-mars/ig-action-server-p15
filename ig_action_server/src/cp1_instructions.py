#! /usr/bin/env python

import rospy
from brass_gazebo_config_manager.srv import *


class CP1_Instructions(object):
    ros_node = '/battery_monitor_client'
    model_name = '/battery_demo_model'

    def __init__(self):
        self.set_configuration_srv = rospy.ServiceProxy(self.ros_node + self.model_name + '/set_robot_configuration', SetConfig)

    def set_config(self, config_id):
        res = self.set_configuration_srv(config_id)
        if res:
            return True, "the new configuration has been set"
        else:
            return False, "the config_id is invalid"