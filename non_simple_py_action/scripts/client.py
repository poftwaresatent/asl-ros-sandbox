#! /usr/bin/env python

import sys
import os
import roslib
roslib.load_manifest('non_simple_py_action')

import rospy
import actionlib
import non_simple_py_action.msg

class CountActionClient (actionlib.action_client.ActionClient):
    def __init__(self, name):
        rospy.loginfo('Creating CountActionClient %s' % name)
        actionlib.action_client.ActionClient.__init__(self, name, non_simple_py_action.msg.CountAction)
        self.counter = dict()
        self.started = False

    def spawn(self, begin, end):
        if not self.started:
            rospy.loginfo('waiting for server...')
            self.started = self.wait_for_server(rospy.Duration(10.0))
            if self.started:
                rospy.loginfo('...OK, server started')
            else:
                rospy.loginfo('...FAILED to start server')
                return False
        goal = non_simple_py_action.msg.CountGoal(begin = begin, end = end)
        self.send_goal(goal, self.transition_cb, self.feedback_cb)
        return True
        
    def transition_cb(self, gh):
        goal = gh.get_goal()
        status = gh.get_goal_status()
        rospy.loginfo('transition %s: from %d to %d' % (status, goal.begin, goal.end))
        if actionlib.action_client.GoalStatus.SUCCEEDED == status:
            rospy.loginfo('  magic %d' % gh.get_result)
            
    def feedback_cb(self, gh, msg):
        goal = gh.get_goal()
        rospy.loginfo('feedback: from %d to %d at %d' % (goal.begin, goal.end, msg.current))

if __name__ == '__main__':
    rospy.init_node('count_client')
    cac = CountActionClient('count')
    cac.spawn(0, 100)
    rospy.Rate(2).sleep()
    cac.spawn(100, -100)
    cac.spawn(0, 10)
    rospy.Rate(0.5).sleep()
    cac.spawn(900, 1100)
    cac.spawn(0, 10)
    cac.spawn(0, 10)
    cac.spawn(0, 10)
    cac.spawn(0, 10)
    rospy.spin()
