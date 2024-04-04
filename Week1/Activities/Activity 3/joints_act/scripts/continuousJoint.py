#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState


# Declare the output Messages


# Declare the output Messages
def init_joints():



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
    rospy.init_node("Puzzlebot_Pose_Estimator")
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/rate",100))
    rospy.on_shutdown(stop)

    #Init joints


    #Setup Transform Broadcasters


    print("The Estimator is Running")
    try:
    #Run the node
        while not rospy.is_shutdown(): 
		#Change state of the joint

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
