#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import numpy as np
from Motor import *            




motos_arry = np.zeros(2,int)
PWM=Motor() 

def motors_callback(data):
    global motos_arry
    # data will contain the message information of type LaserScan, we can access and print that data as follows.
    motos_arry = (data.data)
   # dist = 100
    print(motos_arry)

# PWM=Motor()  
# def test_Motor(): 
#     try:
#         PWM.setMotorModel(1000,1000,1000,1000)       #Forward
#         print ("The car is moving forward")
#         time.sleep(1)
#         PWM.setMotorModel(-1000,-1000,-1000,-1000)   #Back
#         print ("The car is going backwards")
#         time.sleep(1)
#         PWM.setMotorModel(-1500,-1500,2000,2000)       #Left 
#         print ("The car is turning left")
#         time.sleep(1)
#         PWM.setMotorModel(2000,2000,-1500,-1500)       #Right 
#         print ("The car is turning right")  
#         time.sleep(1)
#         PWM.setMotorModel(0,0,0,0)                   #Stop
#         print ("\nEnd of program")
#     except KeyboardInterrupt:
#         PWM.setMotorModel(0,0,0,0)
#         print ("\nEnd of program")

def cont_mot_node():
    global PWM
    global motos_arry
    rospy.init_node('cont_mot_node', anonymous=True)
    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber("/motorsA", Int32MultiArray, motors_callback)
        PWM.setMotorModel(motos_arry[0],motos_arry[0],motos_arry[1],motos_arry[1])
        # PWM.setMotorModel(1000,1000,1000,1000) 
        
        rate.sleep()

def myhook(): # insures safe shutdown and turns of motors
    global PWM
    PWM.setMotorModel(0,0,0,0)
    rospy.loginfo("shutdown.")




if __name__ == '__main__':
    try:
        rospy.on_shutdown(myhook)
        cont_mot_node()
    except rospy.ROSInterruptException:
        pass
