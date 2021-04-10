#!/usr/bin/env python
import rospy
import math
import string
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
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
        
        #Dummy variables
        self.lat0 = rospy.get_param("lat0")
        self.lon0 = rospy.get_param("lon0")
        self.alt0 = rospy.get_param("alt0")
        self.lat_front = 0.0
        self.lon_front = 0.0
        self.alt_front = 0.0
        self.lat_rear = 0.0
        self.lon_rear = 0.0
        self.alt_rear = 0.0

        #Subscribers
        fix_rear_sub = rospy.Subscriber("fix_topic_rear", NavSatFix, self.fix_rear_callback)
        fix_front_sub = rospy.Subscriber("fix_topic_front", NavSatFix, self.fix_front_callback)
        imu_sub = rospy.Subscriber("imu_topic", Imu, self.imu_callback)
        heading_sub = rospy.Subscriber("heading_topic", Vector3Stamped, self.imu_callback)

    
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

    def fix_front_callback(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        self.lat_front = data.latitude
        self.lon_front = data.longitude
        self.alt_front = data.altitude
    
    def fix_rear_callback(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        self.lat_rear = data.latitude
        self.lon_rear = data.longitude
        self.alt_rear = data.altitude

    def calculate_heading(self, gps_rear, gps_front):
        lat1 = math.radians(gps_rear[0])
        lat2 = math.radians(gps_front[0])

        diffLong = math.radians(gps_front[1] - gps_rear[1])

        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                * math.cos(lat2) * math.cos(diffLong))

        initial_bearing = math.atan2(y, x)

        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return initial_bearing

    self.imu_callback(self, msg):
        self.r = msg.angular_velocity.z 

def main():
    # initiate node
    rospy.init_node('HeadingControllerNode',log_level=rospy.DEBUG)
    rospy.loginfo("HeadingControllerNode initiated")
    test = TestNode()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    