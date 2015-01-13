#!/usr/bin/env python

# Copyright (C) 2014, Massachusetts Institute of Technology
# All rights reserved.
#
# This work is sponsored by the Department of the Air Force Air Force
# contract number: FA8721-05-C-0002. The opinions, interpretations,
# recommendations, and conclusions are those of the author and are
# not necessarily endorsed by the United Stated Government.

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class joy_to_servo:
    def __init__(self):

        # get parameters
        self.throttle_axis = rospy.get_param("~throttle_axis", 1)
        self.throttle_positive_gain = rospy.get_param("~throttle_positive_gain", 1)
        self.throttle_negative_gain = rospy.get_param("~throttle_negative_gain", 1)
        self.steering_axis = rospy.get_param("~steering_axis", 1)
        self.steering_positive_gain = rospy.get_param("~steering_positive_gain", 1)
        self.steering_negative_gain = rospy.get_param("~steering_negative_gain", 1)

        # todo: check parameters

        # advertise servo commands
        self.throttle_pub = rospy.Publisher("throttle/command", Float64, queue_size=10)
        self.steering_pub = rospy.Publisher("steering/command", Float64, queue_size=10)

        # subscribe to joy topic
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
		
    def joyCallback(self, joy_msg):
        # get joystick throttle value
        if self.throttle_axis >= len(joy_msg.axes):
            rospy.logerr("Throttle axis index (%s) greater than Joy message axes length (%s), ignoring.",
                         self.throttle_axis, len(joy_msg.axes))
            return
        throttle = joy_msg.axes[self.throttle_axis]
        if throttle > 1 or throttle < -1:
            rospy.logerr("Joy throttle axis value (%s) outside of expected range, ignoring.", throttle)
            return

        # get joystick steering value
        if self.steering_axis >= len(joy_msg.axes):
            rospy.logerr("Steering axis index (%s) greater than Joy message axes length (%s), ignoring.",
                         self.steering_axis, len(joy_msg.axes))
            return
        steering = joy_msg.axes[self.steering_axis]
        if steering > 1 or steering < -1:
            rospy.logerr("Joy steering axis value (%s) outside of expected range, ignoring.", steering)
            return

        # todo; trim axes?

        # apply gains
        if throttle > 0:
            throttle = throttle * self.throttle_positive_gain
        elif throttle < 0:
            throttle = throttle * self.throttle_negative_gain

        if steering > 0:
            steering = steering * self.steering_positive_gain
        elif steering < 0:
            steering = steering * self.steering_negative_gain

        # limit commands
        if throttle > 1:
            throttle = 1
        elif throttle < -1:
            thottle = -1

        if steering > 1:
            steering = 1
        elif steering < -1:
            steering = -1

        # publish servo commands
        self.throttle_pub.publish(throttle)
        self.steering_pub.publish(steering)
		
if __name__ == '__main__':
    rospy.init_node('joy_to_servo')
    node = joy_to_servo()
    rospy.spin()
