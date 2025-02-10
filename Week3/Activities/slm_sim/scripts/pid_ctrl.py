#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

#Define variables to be used
first = True
flag = False
start_time = 0.0 
last_time = 0.0
current_time = 0.0
error = 0.0
error_prev = 0.0
Error_Int = 0.0


# Declare the input Message
system_states = JointState()
torque_msg = Float32()


#Define the callback functions
def callback(msg):
    global system_states, flag
    flag = True
    system_states = msg


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':

    #Initialise node
    rospy.init_node("controller")

    #Set the parameters of the system
    _Kp= rospy.get_param("~Kp",0.0)
    _Ki= rospy.get_param("~Ki",0.0)
    _Kd= rospy.get_param("~Kd",0.0)
    _sampleTime = rospy.get_param("~sample_time",0.01)
    _setPoint = rospy.get_param("~setPoint",0.0)

    #Setup the node
    rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers
    rospy.Subscriber("joint_states", JointState, callback)
    controlInput=rospy.Publisher("torque",Float32,queue_size=1)

    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        #Setup function
        if first == True:
            start_time = rospy.get_time() 
            last_time = rospy.get_time() 
            current_time = rospy.get_time()
            tau = 0.0
            controlInput.publish(torque_msg)
            if flag==True:
                first = False

        #Control
        else:
            #Sample time
            current_time = rospy.get_time()
            dt = current_time - last_time
            if dt >= _sampleTime:

                #Define SetPoints
                time = rospy.get_time()-start_time
                setPoint = _setPoint

                #Error Estimation
                error = setPoint - system_states.position[0]

                #PI controller
                Error_Int += (error) * dt
                tau  = _Kp * error + _Ki *Error_Int - _Kd * (system_states.velocity[0])

                torque_msg.data = tau

                #Publish Control input
                controlInput.publish(torque_msg)
                
                print(system_states.position[0])
                print("Error= ",  error)
                print("dt= ",  dt)
                print(tau)
                
                last_time = rospy.get_time()

        rate.sleep()