#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from geometry_msgs.msg import PoseStamped

# st1_pos = Float32MultiArray()
# PWM=Motor() 
st1_pos = [0.0,0.0,0.0]
st2_pos = [0.0,0.0,0.0]
st3_pos = [0.0,0.0,0.0]
st4_pos = [0.0,0.0,0.0]
st5_pos = [0.0,0.0,0.0]

robota = [0.0,0.0,0.0]
robotb = [0.0,0.0,0.0]


def st1pos_callback(data):
    global st1_pos
    st1_pos = [data.pose.position.x,data.pose.position.y,data.pose.position.z]

def st2pos_callback(data):
    global st2_pos
    st2_pos = [data.pose.position.x,data.pose.position.y,data.pose.position.z]

def st3pos_callback(data):
    global st3_pos
    st3_pos = [data.pose.position.x,data.pose.position.y,data.pose.position.z]

def st4pos_callback(data):
    global st4_pos
    st4_pos = [data.pose.position.x,data.pose.position.y,data.pose.position.z]

def st5pos_callback(data):
    global st5_pos
    st5_pos = [data.pose.position.x,data.pose.position.y,data.pose.position.z]    

def robota_callback(data):
    global robota
    robota = [data.pose.position.x,data.pose.position.y,data.pose.position.z]

def robotb_callback(data):
    global robotb
    robotb = [data.pose.position.x,data.pose.position.y,data.pose.position.z]

def pos_Sub():
    # global PWM
    global st1_pos

    # global motos_arry
    rospy.init_node('pos_Sub', anonymous=True)
    pub_st1 = rospy.Publisher('pos_st1', Float32MultiArray, queue_size=1)
    pub_st2 = rospy.Publisher('pos_st2', Float32MultiArray, queue_size=1)
    pub_st3 = rospy.Publisher('pos_st3', Float32MultiArray, queue_size=1)
    pub_st4 = rospy.Publisher('pos_st4', Float32MultiArray, queue_size=1)
    pub_st5 = rospy.Publisher('pos_st5', Float32MultiArray, queue_size=1)
    pub_robA = rospy.Publisher('pos_robA', Float32MultiArray, queue_size=1)
    pub_robB = rospy.Publisher('pos_robB', Float32MultiArray, queue_size=1)

    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber("/qualisys/st1/pose", PoseStamped, st1pos_callback)
        rospy.Subscriber("/qualisys/st2/pose", PoseStamped, st2pos_callback)
        rospy.Subscriber("/qualisys/st3/pose", PoseStamped, st3pos_callback)
        rospy.Subscriber("/qualisys/st4/pose", PoseStamped, st4pos_callback)
        rospy.Subscriber("/qualisys/st5/pose", PoseStamped, st5pos_callback)
        rospy.Subscriber("/qualisys/robotA/pose", PoseStamped, robota_callback)
        rospy.Subscriber("/qualisys/robotB/pose", PoseStamped, robotb_callback)


        st1_msg = Float32MultiArray()
        st2_msg = Float32MultiArray()
        st3_msg = Float32MultiArray()
        st4_msg = Float32MultiArray()
        st5_msg = Float32MultiArray()
        robA_msg = Float32MultiArray()
        robB_msg = Float32MultiArray()


        st1_msg.data= st1_pos
        st2_msg.data= st2_pos
        st3_msg.data= st3_pos
        st4_msg.data= st4_pos
        st5_msg.data= st5_pos
        robA_msg.data= robota
        robB_msg.data= robotb


        pub_st1.publish(st1_msg)
        pub_st2.publish(st2_msg)
        pub_st3.publish(st3_msg)
        pub_st4.publish(st4_msg)
        pub_st5.publish(st5_msg)
        pub_robA.publish(robA_msg)
        pub_robB.publish(robB_msg)


        # PWM.setMotorModel(motos_arry[0],motos_arry[0],motos_arry[1],motos_arry[1])
        # PWM.setMotorModel(1000,1000,1000,1000) 
        
        rate.sleep()

# def myhook(): # insures safe shutdown and turns of motors
    # global PWM
    # PWM.setMotorModel(0,0,0,0)
    # rospy.loginfo("shutdown.")




if __name__ == '__main__':
    try:
        # rospy.on_shutdown(myhook)
        pos_Sub()
    except rospy.ROSInterruptException:
        pass
