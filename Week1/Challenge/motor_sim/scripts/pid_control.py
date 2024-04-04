#!/usr/bin/env python

import rospy
#Declare libraries to be used (if needed)
#Declare Messages to be used


#Declare Variables/Parameters to be used

#Define callback functions (if required)

#Define other functions (if required)


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Control")
    rate = rospy.Rate(10)

    #Declare other functions if required

    #Setup Publishers and Subscribers

    #Run the node
    try:
        while not rospy.is_shutdown():
            
            ######## Write your node here ##############

            #Wait and repeat
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass    