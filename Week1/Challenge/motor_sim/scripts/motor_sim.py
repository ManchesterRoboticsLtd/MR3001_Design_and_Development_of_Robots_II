import rospy
import numpy as np
from std_msgs.msg import Float32

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message

 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Motor_Sim")
    
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/motorSimRate",200))
    rospy.on_shutdown(stop)

    # Setup the Subscribers

    #Setup de publishers

    print("The Motor is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 

            #########WRITE YOUR CODE HERE ####      

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node