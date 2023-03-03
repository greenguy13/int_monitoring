#!/usr/bin/env python

"""
F-measure

Process:
1. F-measure that gradually drops given a decay function

Inputs:
1. Robot monitors the area (action request) which then would raise the F-level
2. Request from decision_making about current F-level

Output:
1. Notify action request from robot after F-measure is raised to max level
2. Respond to request from decision_making on current F-level

Thus:
1. Action server for the robot, which asks to raise F-level
2. Server to decision_making about current F-level
"""
import math
import rospy
import actionlib
from intermittent_monitoring.msg import monitorAction, monitorFeedback, monitorResult
from intermittent_monitoring.srv import flevel, flevelResponse
from intermittent_monitoring.srv import assignment_notice, assignment_noticeResponse
from std_msgs.msg import Float32


class Area():
    def __init__(self, area, max_fmeasure, decay_rate, restoration, t_operation):
        rospy.init_node('area_' + str(area))
        self.decay_rate = decay_rate
        self.t_operation = t_operation #total duration of the operation
        self.max_fmeasure = max_fmeasure
        self.fmeasure = self.max_fmeasure #initialize at max fmeasure
        self.restore_delay = int(math.ceil(restoration * (self.max_fmeasure-self.fmeasure))) #delay in restoring the F-measure to max level
        self.request_restore_status = None
        self.global_decay_fmeasure = 0

        # Fmeasure publisher
        self.fmeasure_pub = rospy.Publisher("/fmeasure_" + str(area), Float32, queue_size=1)

        #Action server: Raise Fmeasure
        self.restore_fmeasure_action_server = actionlib.SimpleActionServer("restore_fmeasure_action_server_" + str(area), monitorAction, execute_cb=self.raise_fmeasure_cb, auto_start=False)
        self.restore_fmeasure_action_server.start()

        #Service server: Fmeasure
        self.fmeasure_server = rospy.Service("flevel_server_" + str(area), flevel, self.report_flevel_cb)

        #AUCTION
        self.assigned_to_bidder = False #Whether assigned to a bidder

        #SERVICE SERVER to AUCTIONEER regarding assignment status
        ##We can actually ignore this one. We can just cast as string the assigned_to_bidder status so as the F-measure
        """
        Clearing:
        1. Notice of assignment as service server
            > This updates self.assigned_to_bidder
        2. Request for F-measure as service server as well. 
            > If self.assigned_to_bidder is None: return 'None'
            > Else: return f-measure
        """
        self.assignment_notice_server = rospy.Service("assignment_notice_server_" + str(area), assignment_notice, self.assignment_notice_cb)


    def publish_fmeasure(self):
        """
        Publishes F-measure as a topic
        :return:
        """
        self.fmeasure_pub.publish(self.fmeasure)

    def raise_fmeasure_cb(self, goal):
        """
        Callback as action server for restoring F-measure upon request of action client, (which is motion)
        :param goal:
        :return:
        """
        success = True
        monitor_feedback = monitorFeedback()
        monitor_result = monitorResult()
        rate = rospy.Rate(1)
        #Insert here: Request to raise F-measure, PO1: self.request_restore = True.
        self.request_restore_status = 'restoring'

        for i in range(self.restore_delay):
            if self.restore_fmeasure_action_server.is_preempt_requested():
                success = False
                break
            monitor_feedback.current_fmeasure = 'Restoring F-measure...'
            self.restore_fmeasure_action_server.publish_feedback(monitor_feedback)
            rate.sleep()

        self.fmeasure = goal.max_fmeasure
        monitor_result.raised_max = True

        if success:
            self.request_restore_status = 'restored'
            self.assigned_to_bidder = False #Reset assignment of area
            self.restore_fmeasure_action_server.set_succeeded(monitor_result)

    def report_flevel_cb(self, msg):
        """
        Callback as Service Server for F-measure
        :param msg:
        :return:
        """
        if (bool(msg.fmeasure_request) is True) and (self.assigned_to_bidder is False):
            print("Reporting FMeasure", str(self.fmeasure))
            return flevelResponse(str(self.fmeasure))
        else:
            print("No Fmeasure reported")
            return 'None'

    ##AUCTION METHOD
    """
    Method that gives notice to Area that it has been assigned. And so, Area will update its self.assigned_to_bidder = True,
    which in turn would either report F-measure or 'None', depending on whether it has received an assignment. 
    """
    def assignment_notice_cb(self, msg):
        """

        :param msg:
        :return:
        """

        self.assigned_to_bidder = bool(msg.assignment)
        return assignment_noticeResponse(True)


    def decay(self, t):
        """
        Decay function
        :param t:
        :return:
        """
        decayed_f = self.max_fmeasure*(1 - self.decay_rate)**t
        self.global_decay_fmeasure += self.fmeasure - decayed_f # Global decay F-measure
        self.fmeasure = decayed_f

    def run_operation(self, freq_hz=1):
        t = 0
        # PO (for finite?):for duration in range(self.t_operation): #We use this if we want t_operation to be finite
        # Need to ensure that the decay is per second

        rate = rospy.Rate(freq_hz)
        while not rospy.is_shutdown():
            self.publish_fmeasure()
            print("\nArea status:")
            print("Request restore status:", self.request_restore_status)
            print("Assigned to bidder:", self.assigned_to_bidder)
            if self.request_restore_status == None:
                # print("Current F-measure:", self.fmeasure)
                self.decay(t)
                # print("Decayed:", t, self.fmeasure)
                t += 1
            elif self.request_restore_status == 'restoring':
                pass #not decaying because being restored
            elif self.request_restore_status == 'restored':
                #Restore parameters
                # print("Decayed F-measure prior to restoration:", self.global_decay_fmeasure)
                # print("Restored fmeasure:", self.fmeasure)
                # print("time: {}. Restore status: {}".format(t, self.request_restore_status))
                t = 0
                self.request_restore_status = None
                self.global_decay_fmeasure = 0
            rate.sleep()