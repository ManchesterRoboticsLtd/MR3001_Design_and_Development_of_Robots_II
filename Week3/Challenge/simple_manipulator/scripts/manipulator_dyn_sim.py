#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


#Declare Variables/Parameters to be used

# Setup Variables to be used


#Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


#Main Function
if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("DLM_Sim")

    #Get DLM Parameters


    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    print("The SLM sim is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 

	    ## WRITE YOUR CODE HERE        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
