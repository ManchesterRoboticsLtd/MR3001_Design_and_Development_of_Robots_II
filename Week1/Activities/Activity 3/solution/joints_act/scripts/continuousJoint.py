#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState


# Declare the output Messages

#q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.0)
contJoints = JointState()

# Declare the output Messages
def init_joints():
    contJoints.header.frame_id = "link1"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.extend(["joint1", "joint2_1", "joint2_2", "joint2_3", "joint3", "joint4" ])
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
    rospy.init_node("Puzzlebot_Pose_Estimator")
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/rate",100))
    rospy.on_shutdown(stop)

    #Init joints
    init_joints()

    #Setup Transform Broadcasters
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    print("The Estimator is Running")
    try:
    #Run the node
        while not rospy.is_shutdown(): 
            t = rospy.Time.now().to_sec()
            contJoints.header.stamp = rospy.Time.now()
            contJoints.position[0] = wrap_to_Pi(t)
            contJoints.position[1] = wrap_to_Pi(0.5*t)
            contJoints.position[2] = wrap_to_Pi(0.5*t)
            contJoints.position[3] = wrap_to_Pi(0.5*t)
            contJoints.position[4] = wrap_to_Pi(0.1*t)
            contJoints.position[5] = wrap_to_Pi(t)
            
            joint_pub.publish(contJoints)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass