#! /usr/bin/env python

import rospy
from brass_gazebo_config_manager.srv import *
from brass_gazebo_battery.srv import *


class CP1_Instructions(object):
    ros_node = '/battery_monitor_client'
    model_name = '/battery_demo_model'

    def __init__(self):
        self.set_configuration_srv = rospy.ServiceProxy(self.ros_node + self.model_name + '/set_robot_configuration', SetConfig)
        self.set_charging_srv = rospy.ServiceProxy(self.ros_node + self.model_name + '/set_charging', SetCharging)

    def set_config(self, config_id):
        print("Setting configuration to " + config_id)
        res = self.set_configuration_srv(config_id)
        if res:
            return True, "the new configuration has been set"
        else:
            return False, "the config_id is invalid"

    def set_charging(self, charging):
        return self.set_charging_srv(charging)

    def dock(self, seconds):
        rospy.loginfo("The bot is docked and start charging for {0} seconds".format(seconds))
        self.set_charging(1)
        return True

    def undock(self):
        self.set_charging(0)
        rospy.loginfo("The bot is now undocked")
        return True

    def charge(self, seconds):
        self.dock(seconds)
        rospy.sleep(rospy.Duration.from_sec(seconds))
        self.undock()
        return True, "Charging is done"
