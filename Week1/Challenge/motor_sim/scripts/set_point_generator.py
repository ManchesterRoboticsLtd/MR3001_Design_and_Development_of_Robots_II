#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SetPoint_Generator")

    #Declare Variables/Parameters to be used
 
    # Configure the Node
    rate = rospy.Rate(200)

    #Setup Publishers and Subscribers
    
    #Declare initial time
    init_time = rospy.get_time()

    while not rospy.is_shutdown():

        ####SIGNAL GENERATOR CODE HERE ###   

        #Wait and repeat
        rate.sleep()