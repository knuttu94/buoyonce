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

class TestNode(object):
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
        self.pos_front_ned = None
        self.pos_rear_ned = None
        self.pose = None
        self.yaw = None
        self.vel_ned = None

        #Subscribers
        fix_rear_sub = rospy.Subscriber("gps_rear_topic", NavSatFix, self.fix_rear_callback)
        fix_front_sub = rospy.Subscriber("gps_front_topic", NavSatFix, self.fix_front_callback)
        vel_sub = rospy.Subscriber("vel_front_topic", Vector3Stamped, self.vel_callback)
        imu_sub = rospy.Subscriber("imu_topic", Vector3Stamped, self.imu_callback)
        ground_truth_sub = rospy.Subscriber("ground_truth_topic", Odometry, self.ground_truth_callback)


    def fix_front_callback(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        self.lat_front = data.latitude
        self.lon_front = data.longitude
        self.alt_front = data.altitude
        self.pos_front_ned = np.asarray(pm.geodetic2ned(lat, lon, alt, self.lat0, self.lon0, self.alt0))
    
    def fix_rear_callback(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        self.lat_rear = data.latitude
        self.lon_rear = data.longitude
        self.alt_rear = data.altitude
        self.pos_rear_ned = np.asarray(pm.geodetic2ned(lat, lon, alt, self.lat0, self.lon0, self.alt0))
        # print(np.linalg.norm(self.pos_front_ned-self.pos_rear_ned))

    def vel_callback(self, data):
        self.vel_ned = [data.vector.x, (-1)*data.vector.y, (-1)*data.vector.z]
    
    def imu_callback(self, data):
        pass
    def ground_truth_callback(self,data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        heading = self.calculate_heading([self.lat_rear,self.lon_rear], [self.lat_front,self.lon_front])
        print("Heading error:", self.yaw*180/math.pi-heading)
        
    
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
        



def main():
    # initiate node
    rospy.init_node('TestNode')
    test = TestNode()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    