#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState


# Declare the output Messages
contJoints = JointState()
i = 0
sign = 1

# Declare the output Messages
def init_joints():
    contJoints.header.frame_id = "joint1"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.extend(["joint2", "joint3"])
    contJoints.position.extend([0.0, 0.0])
    contJoints.velocity.extend([0.0, 0.0])
    contJoints.effort.extend([0.0, 0.0])


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
    rospy.init_node("Servo_Joint")
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/rate",10))
    rospy.on_shutdown(stop)

    #Init joints
    init_joints()

    #Setup Transform Broadcasters
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    print("Joints are publishing")
    try:
    #Run the node
        while not rospy.is_shutdown():

            contJoints.header.stamp = rospy.Time.now()

            
            contJoints.position[0] = 0.2 * i 
            contJoints.position[1] = -0.785 + 2 * 0.785 * i 
                    

            if i > 1:
                sign = -1
            elif i < 0:
                sign = 1

            i = i + sign*0.1
            joint_pub.publish(contJoints)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass