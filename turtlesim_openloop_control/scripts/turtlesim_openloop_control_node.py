#!/usr/bin/python
#

# This work is sponsored by the Department of the Air Force under Air Force
# Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
# recommendations are those of the author and are not necessarily endorsed by
# the United States Government.

import rospy
from geometry_msgs.msg import Twist

class TurtlesimOpenLoopNode:
    def __init__(self):
        # state variables
        self.counter = 0

        # create a periodic interrupt at a fixed rate
        self.period = 0.1 # 10 Hz
        rospy.Timer(rospy.Duration(self.period), self.timer_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)

    def timer_callback(self, event):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = 0
        cmd_vel_msg.angular.z = 0
        
        # go forward for 3 seconds (30 counts at 10hz) at 1.1
        # meters/second, then turn at 45 deg/sec for 2 seconds (20
        # counts)
        if self.counter < 30:
            cmd_vel_msg.linear.x = 1.1
        else:
            cmd_vel_msg.angular.z = 0.785398163

        # check for end of cycle
        self.counter += 1
        if self.counter >= 50:
            self.counter = 0

        # publish the velocity command message
        self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == "__main__":
    rospy.init_node("turtlesim_openloop_control_node")
    node = TurtlesimOpenLoopNode()
    rospy.spin()
