#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Declare Variables to be used


# Declare the input Message

# Declare the  process output message

#Define the callback functions

# Setup Variables to be used
first = True

# Declare the input Message

# Declare the output message
slmJoints = JointState()

#Define the callback functions


#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get SLM Parameters
        

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param(100))
    rospy.on_shutdown(stop)

    # Setup the Subscribers

    #Setup de publishers
    state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    print("The SLM sim is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True:
                # System parameters

                # Initialise Joint States
                    #Publish initial conditions to the joints (joint2)
                #Initialise time and variables
                first = False

        #System
            else:
            #Define sampling time
        
            #Dynamical System Simulation
               

             #WRITE YOUR CODE HERE
             #WRITE YOUR PENDULUM HERE

            #Publish message
            #PUBLISH TO JOINTS
      

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node