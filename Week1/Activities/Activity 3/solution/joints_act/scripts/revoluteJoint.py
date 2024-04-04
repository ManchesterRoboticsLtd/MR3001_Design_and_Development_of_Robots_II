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
    contJoints.header.frame_id = "link1"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.extend(["joint1", "joint2_1", "joint2_2", "joint2_3", "joint3", "joint4"])
    contJoints.position.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    contJoints.velocity.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    contJoints.effort.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


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
    rospy.init_node("Revolute_Joint")
 
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

            t = rospy.Time.now().to_sec()
            contJoints.position[0] = wrap_to_Pi(0.2*t)

            contJoints.position[1] = -0.785 + 2*0.785*i
            contJoints.position[2] = -1.57 + 3.14 * i
            contJoints.position[3] = 1.57 * i
            contJoints.position[4] = 1.57 * i
            contJoints.position[5] = -1.57 + 3.14 * i
            
            joint_pub.publish(contJoints)

            if i > 1:
                sign = -1
            elif i < 0:
                sign = 1

            i = i + sign*0.1
            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass