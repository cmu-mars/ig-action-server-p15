#! /usr/bin/env python

import rospy
from brass_gazebo_config_manager.srv import *
from brass_gazebo_battery.srv import *
from std_msgs.msg import Float64
import os
import json


class CP1_Instructions(object):
    ros_node = '/battery_monitor_client'
    model_name = '/battery_demo_model'
    config_file = os.path.expanduser("~/cp1/config.json")
    sleep_interval = 5
    battery_charge_tolerance_threshold = 1000

    def __init__(self):
        self.set_configuration_srv = rospy.ServiceProxy(self.ros_node + self.model_name + '/set_robot_configuration', SetConfig)
        self.set_charging_srv = rospy.ServiceProxy(self.ros_node + self.model_name + '/set_charging', SetCharging)

        with open(self.config_file) as db:
            data = json.load(db)
        self.battery_capacity = data['battery_capacity']
        self.charging_rate = data['charging_rate']
        self.track_battery_charge()

    def set_config(self, config_id):
        print("Setting configuration to " + config_id)
        res = self.set_configuration_srv(config_id)
        if res:
            return True, "the new configuration has been set"
        else:
            return False, "the config_id is invalid"

    def set_charging(self, charging):
        return self.set_charging_srv(charging)

    def dock(self):
        rospy.loginfo("The bot is docked and start charging")
        self.set_charging(1)
        return True

    def undock(self):
        self.set_charging(0)
        rospy.loginfo("The bot is now undocked")
        return True

    def charge(self):
        self.dock()
        while not self.is_fully_charged():
            rospy.sleep(rospy.Duration.from_sec(self.sleep_interval))
        self.undock()
        return True, "Charging is done"

    def is_fully_charged(self):
        if abs(self.battery_charge - self.battery_capacity) <= self.battery_charge_tolerance_threshold:
            rospy.loginfo("Battery is fully charged.")
            return True
        else:
            return False

    def get_charge(self, msg):
        self.battery_charge = msg.data

    def track_battery_charge(self):
        """starts monitoring battery and update battery_charge"""
        rospy.Subscriber("/mobile_base/commands/charge_level_mwh", Float64, self.get_charge)
