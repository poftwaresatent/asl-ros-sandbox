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
        try:
            goal = gh.get_goal()
            if goal.begin == goal.end:
                rospy.loginfo('invalid goal: from %d to %d' % (goal.begin, goal.end))
                gh.set_rejected()
            else:
                rospy.loginfo('new goal: from %d to %d' % (goal.begin, goal.end))
                gh.set_accepted()
                self.counter[gh.get_goal_id().id] = (gh, goal.begin)
        except Exception, ee:
            rospy.logerr('OOPS in goalCallback: %s' % ee)
        except:
            rospy.logerr('unknown exception in goalCallback')

    def cancelCallback(self, gh):
        try:
            goal = gh.get_goal()
            id = gh.get_goal_id()
            rospy.loginfo('canceled: from %d to %d (at %d)' % (goal.begin, goal.end, self.counter[id][1]))
            del self.counter[id]
        except Exception, ee:
            rospy.logerr('OOPS in cancelCallback: %s' % ee)
        except:
            rospy.logerr('unknown exception in cancelCallback')
        
    def update(self):
        rospy.loginfo('updating %d counters' % len(self.counter))
        upd = dict()
        tbr = list()
        for ii in self.counter.itervalues():
            (gh, counter) = ii
            id = gh.get_goal_id()
            goal = gh.get_goal()
            if goal.begin < goal.end:
                counter += 1
            else:
                counter -= 1
                rospy.loginfo('  updated: from %d to %d at %d' % (goal.begin, goal.end, counter))
            upd[id] = counter
            feedback = non_simple_py_action.msg.CountFeedback(current = counter)
            gh.publish_feedback(feedback)
            if counter == goal.end:
                rospy.loginfo('  succeeded: from %d to %d' % (goal.begin, goal.end))
                result = non_simple_py_action.msg.CountResult(magic = goal.begin + goal.end)
                gh.publish_result(result)
                gh.set_succeeded()
                tbr += id
        for (id, counter) in upd.iteritems():
            self.counter[id][1] = counter
        for id in tbr:
            del self.counter[id]

if __name__ == '__main__':
    rospy.init_node('count_server')
    cas = CountActionServer('count')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        cas.update()
        rate.sleep()
