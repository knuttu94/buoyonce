#!/usr/bin/env python
import rospy
import math
import string
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
from usv_state_machine.msg import HeadingControllerInput
import pymap3d as pm
import numpy as np

class HeadingController(object):
    '''
        @brief 
        Node that calculates the correct control input given a desired heading reference
        It is based on the first order Nomoto Model found in Fossen ch 7.3
        The control law is given by
            tau = tau_fb + tau_ff
        with fb -> feedback, ff->feedforward
        tau_fb is a standard PID controller, and tau_ff comes from the Nomoto model

        Should get usv-states as input from one not, but uses raw sensor msgs as of now
    '''
    def __init__(self):

        #Subscribers
        heading_sub = rospy.Subscriber("heading_controller_input", HeadingControllerInput, self.input_callback)

    
        #Nomoto model parameters
        T = 1
        K = -1

        #Usv model states
        self.yaw = 0
        self.r = 0
        self.r_dot = 0
        self.prev_timestamp = 0
        self.current_timestamp = rospy.Time.now()

        #Controller parameters
        omega = 1
        zeta = 1
        m = T/K
        d = 1/T
        self.K_p = m*(omega**2)
        self.K_i = omega*self.K_p/10.0
        self.K_d = (2*zeta*omega*m-d)

        #Saturation parameters
        self.r_max = 1000
        self.r_dot_max = 1000
    
        rospy.loginfo("Heading controller initated")
    
    def run(self, run_freq):
        dt = rospy.Time.now()-self.prev_timestamp
        self.prev_timestamp = rospy.Time.now()
    
    def input_callback(self, msg):
        print(msg)
        

def main():
    # initiate node
    rospy.init_node('HeadingControllerNode',log_level=rospy.DEBUG)
    rospy.loginfo("HeadingControllerNode started")
    heading_controller = HeadingController()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    