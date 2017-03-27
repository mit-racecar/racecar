#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

# import some utils.
import numpy as np
import copy as copy

class InterpolateThrottle:
    def __init__(self):

        # Allow our topics to be dynamic.
        self.rpm_input_topic   = rospy.get_param('~rpm_input_topic', '/vesc/commands/motor/unsmoothed_speed')
        self.rpm_output_topic  = rospy.get_param('~rpm_output_topic', '/vesc/commands/motor/speed')

        self.servo_input_topic   = rospy.get_param('~servo_input_topic', '/vesc/commands/servo/unsmoothed_position')
        self.servo_output_topic  = rospy.get_param('~servo_output_topic', '/vesc/commands/servo/position')

        self.max_acceleration = rospy.get_param('/vesc/max_acceleration')
        self.max_rpm = rospy.get_param('/vesc/vesc_driver/speed_max')
        self.min_rpm = rospy.get_param('/vesc/vesc_driver/speed_min')
        self.throttle_smoother_rate = rospy.get_param('/vesc/throttle_smoother_rate')
        self.speed_to_erpm_gain = rospy.get_param('/vesc/speed_to_erpm_gain')

        self.max_servo_speed = rospy.get_param('/vesc/max_servo_speed')
        self.steering_angle_to_servo_gain = rospy.get_param('/vesc/steering_angle_to_servo_gain')
        self.servo_smoother_rate = rospy.get_param('/vesc/servo_smoother_rate')
        self.max_servo = rospy.get_param('/vesc/vesc_driver/servo_max')
        self.min_servo = rospy.get_param('/vesc/vesc_driver/servo_min')

        # Variables
        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        
        self.last_servo = rospy.get_param('/vesc/steering_angle_to_servo_offset')
        self.desired_servo_position = self.last_servo

        # Create topic subscribers and publishers
        self.rpm_output = rospy.Publisher(self.rpm_output_topic, Float64,queue_size=1)
        self.servo_output = rospy.Publisher(self.servo_output_topic, Float64,queue_size=1)
        
        rospy.Subscriber(self.rpm_input_topic, Float64, self._process_throttle_command)
        rospy.Subscriber(self.servo_input_topic, Float64, self._process_servo_command)

        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate)
        rospy.Timer(rospy.Duration(1.0/self.servo_smoother_rate), self._publish_servo_command)

        self.max_delta_rpm = abs(self.speed_to_erpm_gain * self.max_acceleration / self.throttle_smoother_rate)
        rospy.Timer(rospy.Duration(1.0/self.max_delta_rpm), self._publish_throttle_command)
        
        # run the node
        self._run()

        # Keep the node alive
    def _run(self):
        rospy.spin()

    def _publish_throttle_command(self, evt):
        desired_delta = self.desired_rpm-self.last_rpm
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm + clipped_delta
        self.last_rpm = smoothed_rpm         
        # print self.desired_rpm, smoothed_rpm
        self.rpm_output.publish(Float64(smoothed_rpm))
            
    def _process_throttle_command(self,msg):
        input_rpm = msg.data
        # Do some sanity clipping
        input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)
        self.desired_rpm = input_rpm

    def _publish_servo_command(self, evt):
        desired_delta = self.desired_servo_position-self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo         
        self.servo_output.publish(Float64(smoothed_servo))

    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('Throttle_Interpolator')
        p = InterpolateThrottle()
    except rospy.ROSInterruptException:
        pass
