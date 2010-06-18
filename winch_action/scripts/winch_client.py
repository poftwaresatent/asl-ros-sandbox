#!/usr/bin/env python

import sys
import os
import roslib
roslib.load_manifest('winch_action')

import rospy
import actionlib
import winch_action.msg
import Tix

class GoalSlider(Tix.Frame):
    def __init__(self, parent, lower, upper):
        Tix.Frame.__init__(self, parent)
        Tix.Label(self, text = 'goal').pack(side = Tix.TOP, expand = True, fill = Tix.X)
        goal = (lower + upper) / 2.0
        self.goal_var = Tix.DoubleVar(self)
        self.goal_var.set(goal)
        frame = Tix.Frame(self)
        Tix.Label(frame, text = 'goal', width = 6).pack(side = Tix.LEFT)
        self.goal_scale = Tix.Scale(frame, orient = Tix.HORIZONTAL,
                                    variable = self.goal_var, showvalue = 0,
                                    from_ = lower, to = upper,
                                    command = self._UpdateGoalLabel)
        self.goal_scale.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        self.FORMAT = '%5.2f'
        self.goal_label = Tix.Label(frame, text = self.FORMAT % goal, width = 12)
        self.goal_label.pack(side = Tix.LEFT)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.lower = lower
        self.upper = upper

    def _UpdateGoalLabel(self, goal):
        self.goal_label['text'] = self.FORMAT % float(goal)

    def GetGoal(self):
        return self.goal_var.get()


class GoalSetter(Tix.Frame):
    def __init__(self, parent):
        Tix.Frame.__init__(self, parent, bd = 2, relief = Tix.RIDGE)
        frame = Tix.Frame(self)
        self.button = Tix.Button(frame, text = 'SetGoal', command = self.SetGoal)
        self.button.pack(side = Tix.LEFT)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(no status yet)')
        self.label = Tix.Label(frame, textvariable = self.labeltext)
        self.label.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.goal_slider = GoalSlider(self, -100, 100)
        self.goal_slider.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.winch = None
        
    def LazyInit(self):
        if not self.winch:
            self.labeltext.set('creating winch action client')
            self.winch = actionlib.SimpleActionClient('winch', winch_action.msg.WinchTargetAction)
            self.labeltext.set('waiting for winch action server')
            self.winch.wait_for_server()
            self.labeltext.set('SUCCESS')
            
    def SetGoal(self):
#        try:
            self.LazyInit()
            goal = winch_action.msg.WinchTargetGoal(mode=winch_action.msg.WinchTargetGoal.MODE_GOTO,
                                                    depth=self.goal_slider.GetGoal(),
                                                    speed=1)
            self.labeltext.set('sending goal %g' % self.goal_slider.GetGoal())
            self.winch.send_goal(goal)
            self.labeltext.set('waiting for result...')
            self.winch.wait_for_result()
            result = self.winch.get_result()
            self.labeltext.set('final depth: %g' % result.depth)
#        except rospy.Exception, ee:
#            self.labeltext.set('EXCEPTION: %s' % ee)


if __name__ == '__main__':
    try:
        rospy.init_node('winch_client_py_%d' % os.getpid())
        root = Tix.Tk()
        root.title('Winch Action Client')
        
        lframe = Tix.Frame(root, bd = 2, relief = Tix.RIDGE)
        rframe = Tix.Frame(root)
        
        goal_setter = GoalSetter(rframe)
        quitter = Tix.Button(lframe, text = 'QUIT', command = root.quit, bg = '#cc6666')
        
        lframe.pack(side = Tix.LEFT, expand = True, fill = Tix.Y)
        rframe.pack(side = Tix.LEFT, expand = True, fill = Tix.BOTH)
        
        goal_setter.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        
        Tix.Frame(lframe).pack(side = Tix.TOP, expand = True, fill = Tix.BOTH)
        quitter.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        
        root.mainloop()
        
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
