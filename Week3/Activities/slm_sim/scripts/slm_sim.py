#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#Declare Variables/Parameters to be used

# Setup Variables to be used
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0

# Declare the input Message
torque = Float32()

# Declare the  process output message
slmJoints = JointState()

#Define the callback functions
def input_callback(msg):
    global torque
    torque = msg

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
    total_time = rospy.get_time()-start_time
    rospy.loginfo("Total Simulation Time = %f" % total_time)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get SLM Parameters
    _sampleTime = rospy.get_param("~sample_time",0.01)
    _frictionCoef = rospy.get_param("~friction_coeficient",0.01)
    _rodMass = rospy.get_param("~rod_mass",3.0)
    _rodLength= rospy.get_param("~rod_length",0.4)
    _theta_0= rospy.get_param("~intial_angle",0.0)
    _theta_dot_0= rospy.get_param("~initial_angular_vel",0.0)
    _gravity = rospy.get_param("~gravity",9.8)
    

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(stop)

    # Setup the Subscribers
    rospy.Subscriber("torque",Float32,input_callback)

    #Setup de publishers
    state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    print("The SLM sim is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True:
                # System parameters
                a_COM = _rodLength/2.0
                J_inertia = (4/3.0)*(_rodMass*np.power(a_COM,2))

                #Initial states
                x1 = _theta_0
                x2 = _theta_dot_0

                # Initialise Joint States
                slmJoints.header.frame_id = "base"
                slmJoints.header.stamp = rospy.Time.now()
                slmJoints.name = ["joint2"]
                slmJoints.position = [x1]
                slmJoints.velocity = [x2]
                slmJoints.effort = [0.0]
                #state_pub.publish(slmJoints)

                #Initialise time
                start_time = rospy.get_time() 
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False
        #System
            else:
            #Define sampling time
                current_time = rospy.get_time()
                dt = current_time - last_time
        
                #Dynamical System Simulation
                if dt >= _sampleTime:                   

                    x1 += dt * (x2)

                    x2_dot = (1/(J_inertia)) * (- _rodMass * _gravity * a_COM * np.cos(x1) - _frictionCoef * x2 + torque.data)
                    x2 += dt * x2_dot
                
                    #Message to publish
                    slmJoints.header.stamp = rospy.Time.now()
                    slmJoints.position[0] = x1
                    slmJoints.velocity[0] = x2
                    slmJoints.effort[0] = 0.0

                    #Publish message
                    state_pub.publish(slmJoints)

                    #Get the previous time
                    last_time = rospy.get_time()
        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node