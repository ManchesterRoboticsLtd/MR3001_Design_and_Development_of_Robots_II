#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from sensor_msgs.msg import JointState

# Setup Variables to be used
first = True
flag = False
start_time = 0.0 
last_time = 0.0
current_time = 0.0
error = 0.0

# Declare the output Messages
torque_msg = Float32()
sp_theta_msg = Float32()
error_msg = Float32()

# Declare the input Message
system_states = JointState()

#Define the callback functions
def callback(msg):
    global system_states, flag
    flag = True
    system_states = msg

  #wrap to pi function
def wrap_to_Pi(self,theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

    #Stop Condition
def stop(self):
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("CTC_ctrl")

    #Set the parameters of the simulation environment
    _sampleTime = rospy.get_param("~sample_time",0.01)
    _rodMass = rospy.get_param("~rod_mass",3.0)
    _rodLength= rospy.get_param("~rod_length",0.4)
    _gravity = rospy.get_param("~gravity",9.8)
    _Kd= rospy.get_param("~Kd",40)
    _Kp= rospy.get_param("~Kp",400)

    # Setup the Subscribers
    rospy.Subscriber("joint_states", JointState, callback)

    #Setup de publishers
    controlInput=rospy.Publisher("torque",Float32,queue_size=1)
    controlSetPoint=rospy.Publisher("set_point",Float32, queue_size=1)
    controlError=rospy.Publisher("set_point2",Float32, queue_size=1)
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(stop)

    #Node Running
    print("The Controller is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():
            if first == True:

                # System parameters
                a_COM = _rodLength/2.0
                J_inertia = (4/3.0)*(_rodMass*np.power(a_COM,2))

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
                if dt >= _sampleTime and flag == True:                 

                    #Define SetPoints
                    time = rospy.get_time()-start_time
                    set_point_1 = 1 + 0.3 * np.sin(time)
                    set_point_dot_1 = 0.3 * np.cos(time)
                    set_point_ddot_1 = - 0.3 * np.sin(time)

                    #Estimate Errors


                    #Define Control


                    #Publish Control


                    #Publish message
                    controlInput.publish(torque_msg)



                    #Get the previous time
                    last_time = rospy.get_time()

            loop_rate.sleep() 
    except rospy.ROSInterruptException:
        pass