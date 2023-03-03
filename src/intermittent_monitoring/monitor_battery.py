#!/usr/bin/env python

import pickle
import rospy
from std_msgs.msg import Float32

class BatteryLevelListener():
    def __init__(self, robot_no, sleep):
        """
        Listens to battery level
        :param robot_no:
        :param sleep:
        """
        rospy.init_node("battery_listener_" + str(robot_no))
        rospy.sleep(sleep)

        self.battery_sub = rospy.Subscriber('/battery_level_' + str(robot_no), Float32, self.store_battery_level_cb)
        self.array = list()

    def store_battery_level_cb(self, msg):
        """
        Callback for storing subscribed battery level message into an array
        :param msg:
        :return:
        """
        self.array.append(msg.data)

    def save_recorded_array(self, t_operation, filename=None):
        """
        Return the stored array of length t_operation
        :param t_operation:
        :return:
        """
        while True:
            if len(self.array) >= t_operation:
                break
            else:
                pass

        if filename != None:
            with open(filename, 'wb') as f:
                pickle.dump(self.array, f)

        return self.array