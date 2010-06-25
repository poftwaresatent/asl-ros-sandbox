#! /usr/bin/env python

import sys
import os
import roslib
roslib.load_manifest('non_simple_py_action')

import rospy
import actionlib
import non_simple_py_action.msg

class CountActionServer (actionlib.action_server.ActionServer):
    def __init__(self, name):
        rospy.loginfo('Creating CountActionServer %s' % name)
        actionlib.action_server.ActionServer.__init__(self, name,
                                                      non_simple_py_action.msg.CountAction,
                                                      self.goalCallback, self.cancelCallback)
        self.counter = dict()
        
    def goalCallback(self, gh):
        goal = gh.get_goal()
        if goal.begin == goal.end:
            rospy.loginfo('invalid goal: from %d to %d' % (goal.begin, goal.end))
            gh.set_rejected()
        else:
            rospy.loginfo('new goal: from %d to %d' % (goal.begin, goal.end))
            gh.set_accepted()
            self.counter[gh] = goal.begin
        
    def cancelCallback(self, gh):
        goal = gh.get_goal()
        rospy.loginfo('canceled: from %d to %d (at %d)' % (goal.begin, goal.end, self.counter[gh]))
        del self.counter[gh]
        
    def update(self):
        rospy.loginfo('updating %d counters' % len(self.counter))
        tbr = list()
        for (gh, counter) in self.counter:
            goal = gh.get_goal()
            if goal.begin < goal.end:
                counter += 1
            else:
                counter -= 1
                rospy.loginfo('  updated: from %d to %d at %d' % (goal.begin, goal.end, counter))
            feedback = non_simple_py_action.msg.CountFeedback(current = counter)
            gh.publish_feedback(feedback)
            if counter == goal.end:
                rospy.loginfo('  succeeded: from %d to %d' % (goal.begin, goal.end))
                result = non_simple_py_action.msg.CountResult(magic = goal.begin + goal.end)
                gh.publish_result(result)
                gh.set_succeeded()
                tbr += gh
        for gh in tbr:
            del self.counter[gh]

if __name__ == '__main__':
    rospy.init_node('count_server')
    cas = CountActionServer('count')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        cas.update()
        rate.sleep()
