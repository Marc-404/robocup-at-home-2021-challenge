#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import os
import time

class KillerNode():
    def __init__(self):
        rospy.init_node('suturo_terminator', anonymous=True)
        self.start = False
        sub = rospy.Subscriber("/hsrb/command_velocity", Twist, self.move_command_cb)
        rospy.spin()

    def move_command_cb(self, msg):
        if not self.start:
            self.start = True
            time.sleep(300)
            os.system("rosnode kill giskard")

if __name__ == '__main__':
    KillerNode()