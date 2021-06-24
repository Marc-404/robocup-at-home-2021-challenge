#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
import os

class KillerNode():
    def __init__(self):
        rospy.init_node('suturo_terminator', anonymous=True)
        self.start = False
        sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.move_command_cb)
        rospy.spin()

    def move_command_cb(self, msg):
        if not self.start:
            self.start = True
            rospy.sleep(300.0)
            os.system("rosnode kill giskard")

if __name__ == '__main__':
    KillerNode()
