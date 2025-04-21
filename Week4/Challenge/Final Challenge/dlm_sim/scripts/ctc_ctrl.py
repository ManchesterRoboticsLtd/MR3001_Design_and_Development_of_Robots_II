#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension
import numpy as np
from sensor_msgs.msg import JointState

# Declare the input Message
system_states = JointState()
flag = False

#Define the callback functions (if required)
def callback(data):
    global system_states, flag
    flag = True
    system_states = data

    #Stop Condition
def stop(self):
    print("Stopping")


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("CTC_ctrl")
 
    # Configure the Node
    loop_rate = rospy.Rate(200)

    # Setup the Subscribers
    rospy.Subscriber("joint_states", JointState, callback)

    #Setup de publishers
    controlInput=rospy.Publisher("dlm_input",Float32MultiArray, queue_size=1)

    # Declare the output Messages
    u = Float32MultiArray()
    u.layout.dim.append(MultiArrayDimension())
    u.layout.dim[0].label = "width"
    u.layout.dim[0].size = 1
    u.layout.dim.append(MultiArrayDimension())
    u.layout.dim[1].label = "height"
    u.layout.dim[1].size = 2
    u.layout.data_offset = 0



    #Node Running
    print("The Controller is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():

            #################### WRITE YOUR CODE HERE  #########################################################
            
                        #DELETE THE EXAMPLE

            ####################################################################################################


            ###################################  EXAMPLE ON HOW TO SEND AND RECEIVE DATA TO THE DLM (DELETE) #########################
            #Print angular positions and velocities
            if flag == True:
                #Print the Feedback from the system
                #system_states.position[0] -> q_1    and  system_states.position[0] -> q_2  
                #system_states.velocity[0] -> q_dot_1  and  system_states.position[1] -> q_2_dot
                print("q1 = ",system_states.position[0], "q_1_dot = ",system_states.velocity[0], "\n")
                print("q2 = ",system_states.position[1], "q_2_dot = ",system_states.velocity[1], "\n")

                #Set a control input u = [tau_1, tau_2]
                u.data = [5,4]

                #Publish Control input
                controlInput.publish(u)
            ######################################(DELETE)########################

            loop_rate.sleep() 
    except rospy.ROSInterruptException:
        pass