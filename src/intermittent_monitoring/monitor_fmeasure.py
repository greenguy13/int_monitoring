#!/usr/bin/env python

"""
Subscriber to the levels of F-measure
"""
import pickle
import rospy
from std_msgs.msg import Float32



class FMeasureListener():
    def __init__(self, area, sleep):
        """
        Listens to F-measure
        :param duration: Length of operation/duration to store F-measure
        """
        rospy.init_node('fmeasure_listener_' + str(area))
        rospy.sleep(sleep)

        self.fmeasure_sub = rospy.Subscriber('/fmeasure_' + str(area), Float32, self.store_fmeasure)
        self.array = list()

    def store_fmeasure(self, msg):
        """
        Callback for storing subscribed F-measure message into an array
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