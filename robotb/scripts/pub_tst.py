#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import numpy as np


def pub_tst_node():


    # Create a publisher object with Twist
    pub = rospy.Publisher('motorsB', Int32MultiArray, queue_size=1)
    # Declare the node, and register it with a unique name
    rospy.init_node('pub_tst_node', anonymous=True)
    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)
    # This node doesn't have to run all the time, but whenever a message is received, therefore, we can leave it spinning (waiting to wake up whenever a message is available).
    # rospy.spin()
    while not rospy.is_shutdown():

        vel_msg = Int32MultiArray()
        vals = [1000,1000]
        vel_msg.data = vals
        
        pub.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_tst_node()
    except rospy.ROSInterruptException:
        pass