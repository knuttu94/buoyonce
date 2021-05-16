#!/usr/bin/env python
import rospy
import math
import string
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
from usv_state_machine.msg import HeadingControllerInput
import pymap3d as pm
import numpy as np

d2r = np.pi/180

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
        self.reference_sub = rospy.Subscriber("heading_controller_input", HeadingControllerInput, self.input_callback)
        self.tau_pub       = rospy.Publisher("heading_controller_output", Float64)
    
        #Nomoto model parameters
        T = 1
        K = 1

        #Usv model states
        self.prev_timestamp = rospy.Time.now()

        #Controller parameters
        omega_n = 1
        zeta    = 1
        m       = T/K
        d       = 1/T

        self.K_p = m*(omega_n**2)
        self.K_i = omega_n*self.K_p/10.0
        self.K_d = (2*zeta*omega_n*m-d)

        #Discretization matrices
        a = -(2*zeta + 1)*omega_n
        b = a*omega_n
        c = omega_n**3

        self.A_d = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [c, b, a]
        ])
        self.B_d = np.array([
            [0, 0, -c],
        ])

        self.x_d = np.array([0,0,0]).reshape(3,1) #empty state vector, x = [psi_d, r_d, r_dot_d]^T

        #Saturation elements
        self.r_max = 0.5 #[rad/sec]
        self.r_dot_max = 0.1 #[rad/sec^2]

    
        rospy.loginfo("Heading controller initated")

    
    def input_callback(self, msg):
        dt                  = (rospy.Time.now() - self.prev_timestamp).to_sec()
        self.prev_timestamp = rospy.Time.now()
        x_d                 = self.reference_filter(msg.psi_r)
        psi_tilde           = self.normalize_angle((x_d[0,0] - msg.psi))
        r_tilde             = x_d[1,0] - msg.r
        tau_fb              = -(self.K_p*psi_tilde + self.K_i*psi_tilde*dt + self.K_d*r_tilde)
    

        output      = Float64()
        output.data = tau_fb
        self.tau_pub.publish(output) 

    def reference_filter(self, psi_r):
        # Reference filter according to Fossen 2011, p 378
        timestep = 0.1
        x_d_prev = self.x_d
        if(x_d_prev[1,0] > abs(self.r_max)):
            x_d_prev[1,0] = np.sign(x_d_prev[1])*self.r_max
            x_d_prev[2,0] = 0

        if(x_d_prev[2,0] > abs(self.r_dot_max)):
            x_d_prev[2,0] = np.sign(x_d_prev[2])*self.r_max

        #Euler integration x_k+1 = x_k + h*x_dot_k
        self.x_d = x_d_prev + timestep*(self.A_d.dot(x_d_prev) + self.B_d*psi_r)
        return self.x_d

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle
        

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
    