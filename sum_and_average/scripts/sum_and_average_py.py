#!/usr/bin/python
#

# This work is sponsored by the Department of the Air Force under Air Force
# Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
# recommendations are those of the author and are not necessarily endorsed by
# the United States Government.

# import main ROS python library
import rospy

# import the Float32 message type
from std_msgs.msg import Float32

# simple class to contain the node's variables and code
class SumAndAverageNode:
    # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        self.running_sum = 0 # the running sum since start
        self.moving_average_period = 30 # how often to sample the moving average
        self.moving_average_sum = 0 # sum since the last moving average update
        self.moving_average_count = 0 # number of samples since the last moving average update

        # subscribe to the number stream topic
        rospy.Subscriber("number_stream", Float32, self.number_stream_callback)

        # advertise that we'll publish on the sum and moving_average topics
        self.sum_pub = rospy.Publisher("sum", Float32)
        self.moving_average_pub = rospy.Publisher("moving_average", Float32)

        # get moving average period from parameter server (or use default value if not present)
        self.moving_average_period = rospy.get_param('~moving_average_period',
                                                     self.moving_average_period)
        if (self.moving_average_period < 0.5):
            self.moving_average_period = 0.5

        # create the Timer with period self.moving_average_period
        rospy.Timer(rospy.Duration(self.moving_average_period), self.timer_callback)

        # print out a message for debugging
        rospy.loginfo("Created timer with period of %f seconds", self.moving_average_period)

    # the callback function for the number stream topic subscription
    def number_stream_callback(self, msg):
        # add the data to the running sums
        self.running_sum = self.running_sum + msg.data
        self.moving_average_sum = self.moving_average_sum + msg.data

        # increment the moving average counter
        self.moving_average_count += 1

        # create a message containing the running total
        sum_msg = Float32()
        sum_msg.data = self.running_sum

        # publish the running sum message
        self.sum_pub.publish(sum_msg)

    # the callback function for the timer event
    def timer_callback(self, event):
        # create the message containing the moving average
        moving_average_msg = Float32()
        if (self.moving_average_count > 0):
            moving_average_msg.data = self.moving_average_sum / self.moving_average_count
        else:
            moving_average_msg.data = float('NaN')

        # publish the moving average
        self.moving_average_pub.publish(moving_average_msg)

        # reset the moving average
        self.moving_average_sum = 0
        self.moving_average_count = 0

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("sum_and_average_node")

    node = SumAndAverageNode()

    # enter the ROS main loop
    rospy.spin()

