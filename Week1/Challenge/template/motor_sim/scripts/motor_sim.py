import rospy
import numpy as np
from std_msgs.msg import Float32

#Declare Variables/Parameters to be used
sample_time = rospy.get_param("/motorSimSampleTime",0.01)

#Motor Parameters
R = 6.0
L = 0.3
k1 = 0.04
k2 = k1
J = 0.00008
b = 0.00025
m = 0.0


#Initial conditions
omega = 0.0
current = 0.0

# Setup Variables to be used
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0

# Declare the input Message
motorInput = Float32()

# Declare the  process output message
motorOutput = Float32()


#Define the callback functions
def input_callback(msg):
    global motorInput
    motorInput = msg


 #Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    print("Stopping")
    total_time = rospy.get_time()-start_time
    rospy.loginfo("Total Simulation Time = %f" % total_time)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Motor_Sim")
    
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/motorSimRate",200))
    rospy.on_shutdown(stop)

    # Setup the Subscribers
    rospy.Subscriber("/motor_input",Float32,input_callback)

    #Setup de publishers
    motor_pub = rospy.Publisher("/motor_output", Float32, queue_size=1)

    print("The Motor is Running")
    try:
        #Run the node
        while not rospy.is_shutdown(): 
            if first == True:
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
                if dt >= sample_time:                   
                    #Motor governing equations
                    current += (-(R/L)*current-(k1/L)*omega+(1/L)*motorInput.data)*dt
                    omega += ((k2/J)*current-(b/J)*omega-(1/J)*m)*dt

                    #Message to publish
                    motorOutput.data = omega

                    #Publish message
                    motor_pub.publish(motorOutput)

                    #Get the previous time
                    last_time = rospy.get_time()
        

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node