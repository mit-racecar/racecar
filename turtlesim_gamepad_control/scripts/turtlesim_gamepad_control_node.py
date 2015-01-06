#!/usr/bin/python
#

# This work is sponsored by the Department of the Air Force under Air Force
# Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
# recommendations are those of the author and are not necessarily endorsed by
# the United States Government.

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TurtlesimGamepadNode:
    def __init__(self):
        self.linear_scale = 2.0
        self.angular_scale = 2.0

        # subscribe to the joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # advertise that we'll publish on the turtlesim command velocity topic
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)

    def joy_callback(self, joy_msg):
        # simple check for invalid joy_msg
        if len(joy_msg.axes) < 2:
            rospy.logerror("joy_msg axes array length (%d) has less than expected length (2)", len(joy_msg.axes))
            return

        # convert the joystick message to a velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = joy_msg.axes[1]*self.linear_scale
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = 0
        cmd_vel_msg.angular.z = joy_msg.axes[0]*self.angular_scale

        # publish the velocity command message
        self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == "__main__":
    rospy.init_node("turtlesim_gamepad")
    node = TurtlesimGamepadNode()
    rospy.spin()
