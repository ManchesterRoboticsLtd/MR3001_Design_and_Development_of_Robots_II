#!/usr/bin/env python
import rospy
import numpy as np
from first_order_sys_sim.msg import system_output
from first_order_sys_sim.msg import system_input
from controller_simple.msg import set_point
from std_msgs.msg import Float32

#Define variables to be used
first = True
start_time = 0.0 
last_time = 0.0
current_time = 0.0
error = 0.0
Error_Int = 0.0

u = system_input()
angularVelocity = system_output()
setPoint = set_point()

#Setup callback functions
def callback_feedback(msg):
    global angularVelocity
    angularVelocity = msg

def callback_set_point(msg):
    global setPoint
    setPoint = msg

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")
  u.input = 0.0
  u.stamp = rospy.Time.now()
  controlInput.publish(u)

if __name__=='__main__':

    #Initialise node
    rospy.init_node("controller")

    #Set the parameters of the system
    kp = rospy.get_param("~controller_kp",0.095)
    ki = rospy.get_param("~controller_ki",0.05)
    sample_time = rospy.get_param("~controller_sample_time",0.02)
    u_min = rospy.get_param("~controller_limit_min",-1)
    u_max = rospy.get_param("~controller_limit_max",1)

    #Setup the node
    rate = rospy.Rate(rospy.get_param("~controller_node_rate",100))
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers
    rospy.Subscriber("system_output", system_output, callback_feedback)
    rospy.Subscriber("set_point", set_point, callback_set_point)
    controlInput=rospy.Publisher("system_input",system_input, queue_size=1)

    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():

        #Setup function
        if first == True:
            start_time = rospy.get_time() 
            u.input = 0.0
            u.stamp = rospy.Time.now()
            controlInput.publish(u)
            first = False

        #Control
        else:
            #Sample time
            current_time = rospy.get_time()
            dt = current_time - last_time
            if dt >= sample_time:
                #Error Estimation
                error = setPoint.setpoint - angularVelocity.output
                #PI controller
                Error_Int += (error) * dt
                u.input  = kp * error +ki *Error_Int
                u.stamp = rospy.Time.now()

                if u.input < u_min:
                    u.input = u_min
                if u.input > u_max:
                    u.input = u_max
                #Publish Control input
                controlInput.publish(u)
                
                last_time = rospy.get_time()

        rate.sleep()