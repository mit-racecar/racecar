#!/usr/bin/env python
import rospy

#from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

# import some utils.
import numpy as np
import copy as copy


class InterpolateThrottle:
    def __init__(self):

        # Allow our topics to be dynamic.
        self.input_topic = rospy.get_param('~output_topic', '/vesc/commands/motor/unsmoothed_speed')
        self.output_topic = rospy.get_param('~input_topic', '/vesc/commands/motor/speed')
	self.max_delta_rpm = rospy.get_param('/vesc/vesc_driver/max_delta_rpm')
	self.max_rpm = rospy.get_param('/vesc/vesc_driver/speed_max')
	self.min_rpm = rospy.get_param('/vesc/vesc_driver/speed_min')

        # Create topic subscribers and publishers
        self.output = rospy.Publisher(self.output_topic, Float64,queue_size=1)
        rospy.Subscriber(self.input_topic, Float64, self._process_throttle_command)
        
	# Variables
	self.last_rpm = 0

	# run the node
        self._run()

    # Keep the node alive
    def _run(self):
        rospy.spin()
            
    def _process_throttle_command(self,input):
        input_rpm = input.data

	# Do some sanity clipping
	input_rpm = min(max(input_rpm, self.min_rpm), self.max_rpm)

	desired_delta = input_rpm-self.last_rpm
	# Bound the delta
	clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
	smoothed_rpm = self.last_rpm + clipped_delta
	self.last_rpm = smoothed_rpm                
	self.output.publish(Float64(smoothed_rpm))

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('Throttle_Interpolator')
        p = InterpolateThrottle()
    except rospy.ROSInterruptException:
        pass
