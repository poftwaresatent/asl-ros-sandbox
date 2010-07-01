#! /usr/bin/env python

import sys
import os
import roslib
roslib.load_manifest('non_simple_action')

import rospy
import actionlib
import non_simple_action.msg

if __name__ == '__main__':
    rospy.init_node('count_client')
    sac = actionlib.SimpleActionClient('count',
                                       non_simple_action.msg.CountAction)
    print 'waiting for server'
    sac.wait_for_server()
    print 'sending goal'
    goal = non_simple_action.msg.CountGoal(begin = -10, end = 10)
    sac.send_goal(goal)
    print 'waiting for result'
    sac.wait_for_result()
    result = sac.get_result()
    print 'result: %s' % result
