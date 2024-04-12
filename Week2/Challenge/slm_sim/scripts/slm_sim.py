#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#Declare Variables to be used

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message


#Define the callback functions


  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    # Setup the Subscribers


    #Setup de publishers

    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node