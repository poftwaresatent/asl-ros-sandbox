#! /usr/bin/env python

import sys
import os
import threading
import roslib
roslib.load_manifest('non_simple_py_action')

import rospy
import actionlib
import non_simple_py_action.msg


class Spinner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        rospy.spin()


class Counter:
    def __init__(self, goalhandle):
        self.gh = goalhandle
        self.id = self.gh.get_goal_id()
        goal = self.gh.get_goal()
        self.begin = goal.begin
        self.end = goal.end
        self.value = self.begin


class CountActionServer (actionlib.action_server.ActionServer):
    def __init__(self, name):
        rospy.loginfo('Creating CountActionServer %s' % name)
        actionlib.action_server.ActionServer.__init__(self, name, \
                              non_simple_py_action.msg.CountAction,
                              self.goalCallback, self.cancelCallback)
        self.counters = dict()
        
    def goalCallback(self, gh):
        try:
            goal = gh.get_goal()
            if goal.begin == goal.end:
                rospy.loginfo('invalid goal: from %d to %d'
                              % (goal.begin, goal.end))
                gh.set_rejected()
            else:
                rospy.loginfo('new goal: from %d to %d' % (goal.begin, goal.end))
                gh.set_accepted()
                counter = Counter(gh)
                self.counters[counter.id] = counter
        except Exception, ee:
            rospy.logerr('OOPS in goalCallback: %s' % ee)
        except:
            rospy.logerr('unknown exception in goalCallback')

    def cancelCallback(self, gh):
        try:
            goal = gh.get_goal()
            id = gh.get_goal_id()
            if not self.counters.has_key(id):
                raise RuntimeError('unknown goal ID %s' % id)
            rospy.loginfo('canceled: from %d to %d (at %d)'
                          % (goal.begin, goal.end, self.counter[id][1]))
            del self.counters[id]
        except Exception, ee:
            rospy.logerr('OOPS in cancelCallback: %s' % ee)
        except:
            rospy.logerr('unknown exception in cancelCallback')
        
    def update(self):
        rospy.loginfo('updating %d counters' % len(self.counters))
        tbr = list()
        for counter in self.counters.itervalues():
            if counter.begin < counter.end:
                counter.value += 1
            else:
                counter.value -= 1
            rospy.loginfo('  updated: from %d to %d at %d'
                          % (counter.begin, counter.end, counter.value))
            counter.gh.publish_feedback(non_simple_py_action.msg.CountFeedback( \
                    current = counter.value))
            if counter.value == counter.end:
                rospy.loginfo('  succeeded: from %d to %d'
                              % (counter.begin, counter.end))
                counter.gh.set_succeeded(non_simple_py_action.msg.CountResult( \
                        magic = counter.begin + counter.end))
                tbr.append(counter.id)
        for id in tbr:
            del self.counters[id]

if __name__ == '__main__':
    rospy.init_node('count_server')
    rospy.loginfo('starting spinner thread')
    spinner = Spinner()
    spinner.start()
    rospy.loginfo('starting action server')
    cas = CountActionServer('count')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        cas.update()
        rate.sleep()
