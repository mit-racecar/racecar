#!/usr/bin/python
#

# This work is sponsored by the Department of the Air Force under Air Force
# Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
# recommendations are those of the author and are not necessarily endorsed by
# the United States Government.

import random
import rospy
from std_msgs.msg import Float32

class RandomNumberStream:
    def __init__(self):
        self.period = 0.1
        self.bias = 0.25

        self.number_stream_pub = rospy.Publisher("number_stream", Float32)

        self.period = rospy.get_param('~period', self.period)
        if (self.period < 0.01):
            self.period = 0.01
        self.bias = rospy.get_param('~bias', self.bias)

        rospy.Timer(rospy.Duration(self.period), self.timer_callback)

    def timer_callback(self, event):
        number_stream_msg = Float32()
        number_stream_msg.data = random.random()-0.5 + self.bias
        self.number_stream_pub.publish(number_stream_msg)

if __name__ == "__main__":
    rospy.init_node("random_number_stream")
    node = RandomNumberStream()
    rospy.spin()

