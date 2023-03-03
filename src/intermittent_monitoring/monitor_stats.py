#!/usr/bin/env python

"""
Subscriber to the levels of F-measure
"""
import pickle
import rospy
from std_msgs.msg import Float32



class StatsListener():
    def __init__(self, areas, robots, sleep):
        """
        Listens to F-measure of all the areas plus the battery of robots
        :param areas: list areas to monitor
        :param robots: list robots whose battery to monitor
        :param sleep: int buffer sleep
        """
        rospy.init_node('stats_listener')
        rospy.sleep(sleep)
        self.areas = areas
        self.robots = robots
        self.sub_fmeasure_dict = dict() #dictionary for subscriber F-measure of areas
        self.sub_battery_dict = dict() #dictionary for subscriber battery of robots

        self.results_fmeasure_dict = dict() #dictionary for stored F-measure data
        self.results_battery_dict = dict() #dictionary for stored battery data

        for area in self.areas:
            self.results_fmeasure_dict[area] = list()
        for robot_no in self.robots:
            self.results_battery_dict[robot_no] = list()
        for area in self.areas:
            self.sub_fmeasure_dict[area] = rospy.Subscriber('/fmeasure_' + str(area), Float32, self.store_fmeasure_cb, area)
        for robot_no in self.robots:
            self.sub_battery_dict[robot_no] = rospy.Subscriber('/battery_level_' + str(robot_no), Float32, self.store_battery_level_cb, robot_no)

    def get_length_stored_data(self):
        """
        Gets the length of stored data so far
        :return:
        """
        sample_area = self.area[0]
        len_fmeasure_data = len(self.results_fmeasure_dict[sample_area]) #length of stored f-measure so far
        len_battery_data = len(self.results_battery_dict[sample_area]) #stored battery
        length = min(len_fmeasure_data, len_battery_data)
        return length

    def store_fmeasure_cb(self, msg, area):
        """
        Callback for storing subscribed F-measure message into an array
        :param msg:
        :return:
        """
        self.results_fmeasure_dict[area].append(msg.data)

    def store_battery_level_cb(self, msg, robot_no):
        """
        Callback for storing subscribed battery level message into an array
        :param msg:
        :return:
        """
        self.results_battery_dict[robot_no].append(msg.data)

    def save_recorded_array(self, t_operation, filename=None):
        """
        Return the stored array of length t_operation
        :param t_operation:
        :return:
        """
        while True:
            fmeasure_counter = 0 #counter if areas have reached the desired number of stored data
            battery_counter = 0 #counter for stored battery

            for area in self.areas:
                #print("Area: {} Data count: {}".format(len(self.results_fmeasure_dict[area])))
                if len(self.results_fmeasure_dict[area]) == t_operation:
                    fmeasure_counter += 1

            for robot_no in self.robots:
                #print("Robot: {} Data count: {}".format(len(self.results_battery_dict[robot_no])))
                if len(self.results_battery_dict[robot_no]) == t_operation:
                    battery_counter += 1

            if (fmeasure_counter == len(self.areas)) and (battery_counter == len(self.robots)):
                break
            else:
                pass

        results = {'fmeasures': self.results_fmeasure_dict,
                   'battery': self.results_battery_dict}

        if filename != None:
            with open(filename, 'wb') as f:
                pickle.dump(results, f)

        #INSERT: We can insert the shutdown here

        return results