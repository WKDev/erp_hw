# 230313 kbub-chson
# odometry manager
# takes the serial data of erp42 as input, returns its odometry
import math
from erp_msgs.msg import ErpData
from nav_msgs.msg import Odometry
# from tf.transformations import quaternion_from_euler
import random as rd
from geometry_msgs.msg import TransformStamped
import numpy as np
import time
from sensor_msgs.msg import JointState
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
import traceback
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class OdometryNode(Node):
    def __init__(self):
        Node.__init__(self,'erp_odometry_node')

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.steer_angle = 0.0
        self.delta_encoder = 0
        self.encoder = 0
        self.last_encoder = 0
        self.wheel_pos = 0.0
        self.lg = self.get_logger() # logger
        self.lg.info('odom_calculator...')

        self.wheel_radius = self.declare_parameter('wheel_radius', 0.265).get_parameter_value().double_value
        self.wheel_base= self.declare_parameter('wheel_base', 1.040).get_parameter_value().double_value
        self.wheel_tread =self.declare_parameter('wheel_tread', 0.985).get_parameter_value().double_value
        self.max_vel = self.declare_parameter('max_vel', 5.0).get_parameter_value().double_value
        self.min_vel= self.declare_parameter('min_vel', -5.0).get_parameter_value().double_value
        self.max_steer_angle = self.declare_parameter('max_steer_angle', 28.169).get_parameter_value().double_value
        self.min_steer_angle = self.declare_parameter('min_steer_angle', -28.169).get_parameter_value().double_value

        # self.odom_pub = rospy.Publisher('/erp42_test/odom',Odometry,queue_size=10)
        # self.serial_sub = rospy.Subscriber('erp_data', ErpData, self.CalculateOdometry)

        # self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        # self.t1 = time.time()


        self.serial_sub = self.create_subscription(
            ErpData,
            'ErpData',
            self.calculate_odometry,
            10)
        self.serial_sub  # prevent unused variable warning


        self.publisher_ = self.create_publisher(Odometry, 'erp_odom', 10) # declare publisher
        pub_rate = 100  # seconds, 100hz


        self.reset_odometry()

        self.timer = self.create_timer(1/pub_rate, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        self.lg.info('publishing...')
        # self.joint_state_pub.publish(joint_state_msg)
        self.odom_pub.publish(self.odom)

    def calculate_odometry(self, data):
        if type(data)!=float:
            delta_time = time.time()-self.t1
            self.t1 = time.time()

            self.lg.info('calculating odometry....')

            # 엔코더 변화량 측정
            self.delta_encoder = data.encoder - self.last_encoder
            self.last_encoder = data.encoder

            # 시간 주기동안 얼마나 굴러간 각도
            self.wheel_pos = self.TICK2RAD * self.delta_encoder

            # 각도 --> 길이(반지름* 각도)
            self.delta_pos = self.wheel_radius * self.wheel_pos

            # 선속도 
            self.linear_vel = self.delta_pos / delta_time
            
            # 각속도 
            self.angular_vel = math.tan(np.deg2rad(data.steer)) * self.linear_vel / self.wheel_base

            self.odom_yaw += self.angular_vel/np.pi()
                    
            self.odom_x += self.delta_pos * math.cos(-np.deg2rad(self.odom_yaw))
            self.odom_y += self.delta_pos * math.sin(-np.deg2rad(self.odom_yaw))
            self.lg.info("Previous Encoder: %d", self.last_encoder)
            self.lg.info("Delta Pose: %f", self.delta_pos)
            self.lg.info("Linear Vel: %f", self.linear_vel)
            self.lg.info("Angular Vel: %f", self.angular_vel)
            self.lg.info("Odom X: %f", self.odom_x)
            self.lg.info("Odom Y: %f", self.odom_y)
            self.lg.info("Odom Yaw: %f", self.odom_yaw)

            odom_quat = quaternion_from_euler(0, 0, -np.deg2rad(self.odom_yaw))


            if math.isnan(self.delta_pos):
                self.delta_pos = 0.0
            if math.isnan(self.angular_vel):
                self.angular_vel = 0.0

            # self.odom.child_frame_id='base_link'
            # self.odom.pose.pose.position.x = 


            # Create an Odometry message
            self.odom.header.stamp = self.get_clock().now().to_msg()
            self.odom.header.frame_id = 'odom'
            self.odom.child_frame_id = 'base_link'

            # Set the position and orientation of the Odometry message,
            # based on the latest transform
            self.odom.pose.pose.position.x = self.odom_x
            self.odom.pose.pose.position.y = self.odom_y
            self.odom.pose.pose.orientation.x = odom_quat[0]
            self.odom.pose.pose.orientation.y = odom_quat[1]
            self.odom.pose.pose.orientation.z = odom_quat[2]
            self.odom.pose.pose.orientation.w = odom_quat[3]
            
            # Set the velocity of the Odometry message (in the 'base_link' frame),
            # based on the data from the erp42_interface
            self.odom.twist.twist.linear.x = self.linear_vel
            self.odom.twist.twist.linear.y = 0 # since the vehicle cannot move like a crab.
            self.odom.twist.twist.angular.z = self.angular_vel

            # ## publishing tf
            # br = tf2_ros.TransformBroadcaster()
            # t = TransformStamped()

            # t.header.stamp = rospy.Time.now()
            # t.header.frame_id = "odom"
            # t.child_frame_id = "base_link"
            # t.transform.translation.x = self.odom_x
            # t.transform.translation.y = self.odom_y
            # t.transform.translation.z = 0.0
            # # q = tf_conversions.transformations.quaternion_from_euler(0, 0, )
            # t.transform.rotation.x = odom_quat[0]
            # t.transform.rotation.y = odom_quat[1]
            # t.transform.rotation.z = odom_quat[2]
            # t.transform.rotation.w = odom_quat[3]

            # br.sendTransform(t)


            # ## publishing tf
            # joint_state_msg = JointState()
            # joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            # joint_state_msg.name = ['front_right_steer_joint','front_right_wheel_joint', 'front_left_steer_joint','front_left_wheel_joint','front_steer_joint','rear_right_wheel_joint', 'rear_left_wheel_joint','rear_wheel_joint']
            # # rear_wheel_state = self.odom_x % (2*3.141592627)
            # rear_wheel_state = data.encoder

            # rad_steer = -np.deg2rad(data.steer)
            # joint_state_msg.position= [rad_steer,rad_steer,rad_steer,rad_steer,rad_steer,rear_wheel_state,rear_wheel_state,rear_wheel_state]
            # # joint_state_msg.velocity= [0,0,0,0,0,self.linear_vel,self.linear_vel,self.linear_vel]


    def reset_odometry(self):
        self.set_odometry(0.0, 0.0, 0.0,0.0,0.0,0.0)
        self.odom_pub.publish(self.odom)

    def set_odometry(self, x, y, z, r,p,yaw):
        self.odom.pose.pose.position.x = x
        self.odom.pose.pose.position.y = y
        self.odom.pose.pose.position.z = z
        q =quaternion_from_euler(r,p,yaw)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    odometry_node = OdometryNode()

    rclpy.spin(odometry_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry_node.destroy_node()
    rclpy.shutdown()
            
if __name__ == "__main__":
    main()
