# 230516 chson
# tf broadcaster
# ros2 foxy

import threading
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
import traceback
from nav_msgs.msg import Odometry


import math

from geometry_msgs.msg import TransformStamped

import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


## Prerequisites of EBIMU Settings: 
# quaternion mode
# linacc
# batt enable
# temp enable


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Declare and acquire `turtlename` parameter
        self.erp_name = self.declare_parameter(
          'erp_name', 'erp').get_parameter_value().string_value
        
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.265).get_parameter_value().double_value
        self.wheel_base= self.declare_parameter('wheel_base', 1.040).get_parameter_value().double_value
        self.wheel_tread =self.declare_parameter('wheel_tread', 0.985).get_parameter_value().double_value
        self.max_vel = self.declare_parameter('max_vel', 5.0).get_parameter_value().double_value
        self.min_vel= self.declare_parameter('min_vel', -5.0).get_parameter_value().double_value
        self.max_steer_angle = self.declare_parameter('max_steer_angle', 28.169).get_parameter_value().double_value
        self.min_steer_angle = self.declare_parameter('min_steer_angle', -28.169).get_parameter_value().double_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Odometry,
            f'/{self.erp_name}/pose',
            self.handle_turtle_pose,
            1)
        self.subscription  # prevent unused variable warning

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.erp_name

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
