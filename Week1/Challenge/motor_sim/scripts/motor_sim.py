import rospy
import numpy as np
from std_msgs.msg import Float32

#Initial conditions

# Setup Variables to be used
first = True
current_time = 0.0
last_time = 0.0

# Declare the input Message

# Declare the  process output message

#Define the callback functions

 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Motor_Sim")
    

    #Declare Variables/Parameters to be used
    sample_time = 0.01

    #Motor Parameters
    R = 6.0
    L = 0.3
    k1 = 0.04
    k2 = 0.04
    J = 0.00008
    b = 0.00025
    m = 0.00

    # Configure the Node
    loop_rate = rospy.Rate(200)
    rospy.on_shutdown(stop)

    # Setup the Subscribers

    #Setup de publishers

    print("The Motor is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True:
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False
        #System
            else:
            #Define sampling time
                current_time = rospy.get_time()
                dt = current_time - last_time
        
                #Dynamical System Simulation
                if dt >= sample_time:    
                    ###### YORU CODE HERE ###########               
                    #Motor governing equations

                    #Message to publish

                    #Publish message

                    #Get the previous time
                    last_time = rospy.get_time()
        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node