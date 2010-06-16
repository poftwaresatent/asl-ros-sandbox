#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('asltutorial')

import rospy
from asltutorial import srv
from asltutorial import msg

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
        self.set_goal = None
        
    def LazyInit(self):
        if not self.set_goal:
            service_name = '/go_server/set_goal'
            self.labeltext.set('waiting for %s' % service_name)
            rospy.wait_for_service(service_name)
            self.set_goal = rospy.ServiceProxy(service_name, srv.SetGoal)
            self.labeltext.set('SUCCESS')
        
    def SetGoal(self):
        try:
            self.LazyInit()
            goal = self.goal_slider.GetGoal()
            response = self.set_goal(goal)
            if not response.ok:
                self.labeltext.set('%s' % response.errstr)
            else:
                self.labeltext.set('ok')
        except rospy.ServiceException, ee:
            self.labeltext.set('EXCEPTION: %s' % ee)


if __name__ == '__main__':
    root = Tix.Tk()
    root.title('ASL ROS tutorial mixer')
    
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
