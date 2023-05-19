#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters

import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
# import tf listener
import tf2_ros

from rclpy.qos import qos_profile_sensor_data
class TrackedObjects:
    def __init__(self, node):
        self.node = node
        self.timestamp = None

        self.qos_profile = qos_profile_sensor_data

        self.laser_back_sub = node.create_subscription(LaserScan, 'itav_agv/safety_lidar_front_link/scan', self.laser_callback, qos_profile_sensor_data)

        self.point2_pub = node.create_publisher(PointCloud2, 'joint_points', 1)
    
    def laser_callback(self, msg):

        lp = lg.LaserProjection()

        pc = lp.projectLaser(msg)
        
        self.point2_pub.publish(pc)

        



def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tracked_objects')

    tracked_objects = TrackedObjects(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()