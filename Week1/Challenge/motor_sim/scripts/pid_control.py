#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


#Declare Variables to be used
first = True
start_time = 0.0 
last_time = 0.0
current_time = 0.0

#Initialise message to be published

#Initialise messages for the subscribers

#Define callback functions

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Control")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Declare Parameters to be used
    sample_time = 0.01

    #Setup Publishers and Subscribers

    
    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():
        
        #Setup the variables (run only one time)
        if first == True:
            start_time = rospy.get_time() 
            last_time = rospy.get_time()
            current_time = rospy.get_time()
            first = False

        #Controller
        else:
            current_time = rospy.get_time()
            dt = current_time - last_time       #Get the sampling time

            #Get the control output, using the motor angular speed and set point
            if dt >= sample_time:
         
               ############ WRITE YOUR CONTROLLER HERE  #########################

                #Get Previous time          
                last_time = rospy.get_time()

        #Wait and repeat
        rate.sleep()