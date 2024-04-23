#!/usr/bin/env python
import rospy
import numpy as np
from controller_simple.msg import set_point

# Setup Variables and messages to be used
r = set_point()

#Define variables to be used
first = True
start_time = 0.0 
signal = 0.0
time = 0.0

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")
  r.setpoint = 0.0
  r.stamp = rospy.Time.now()
  signal_pub.publish(r)

if __name__=='__main__':

    #Initialise the node
    rospy.init_node("Set_Point_Generator")

    #Set the parameters of the system
    Amplitude = rospy.get_param("~setPoint_amplitude",8.0)
    Omega = rospy.get_param("~setPoint_angularfreq",0.1)

    #Setup the node  
    rate = rospy.Rate(rospy.get_param("~setPoint_node_rate",10))
    rospy.on_shutdown(stop)

    #Publishers and subscribers
    signal_pub=rospy.Publisher("set_point",set_point, queue_size=1)

    print("The Set Point Genertor is Running")

    #Run the node
    while not rospy.is_shutdown():
        #Setup Function
        if first == True:
            start_time = rospy.get_time()
            r.setpoint = 0.0
            r.stamp = rospy.Time.now()
            signal_pub.publish(r)
            first = False

        #Signal Generator
        else:
            time = rospy.get_time()-start_time
            signal = Amplitude*np.sin(Omega*time)

            #Publish signal
            r.setpoint = signal
            r.stamp = rospy.Time.now()
            signal_pub.publish(r)

        rate.sleep()