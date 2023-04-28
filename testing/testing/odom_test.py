import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

import os
import time
import datetime
import pickle

A_VEL = 0.0
F_VEL = 5.0


class OdomTest(Node):
    
    def __init__(self, test_name=None):
        super().__init__('odom_test')

        self.declare_parameter('odom_topic', 'odometry/global')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        
        self.testname = test_name

        self.setup_sub_publisher()

        self.file_location = "/home/daniel/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/"

        #Check if file location exists
        if not os.path.exists(self.file_location):
            print("File location does not exist, quitting.")
            exit()
        else:
            print("File location exists, saving data to: " + self.file_location)

        self.reached_speed = False

        # intialize an array of Odometry type messages
        self.odom_msgs = []
        self.odom_gt_msgs = []




    

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.twist.twist.linear.x)


    def setup_sub_publisher(self):
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        
        self.odom_gt_subscription = self.create_subscription(
            Odometry,
            '/wagon/base_link_pose_gt',
            self.odom_gt_callback,
            10)
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
         
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)




    def forward_test(self, f_vel, r_vel):

        vel_msg = Twist()
        vel_msg.linear.x = f_vel
        vel_msg.angular.z = r_vel
        self.cmd_vel_publisher.publish(vel_msg)

    def odom_callback(self, msg):
        
        self.odom_msgs.append(msg)

    def odom_gt_callback(self, msg):
        if self.reached_speed:
            self.odom_gt_msgs.append(msg)
        elif msg.twist.twist.linear.x >= self.cmd_vel_msg.linear.x * 0.9:
            print("reached speed")
            self.reached_speed = True

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def get_reached_speed(self):
        return self.reached_speed
    
    def save_data(self):

        with open(self.file_location + str(self.testname) + "_f_vel_" + str(F_VEL) + "_a_vel_" + str(A_VEL) + ".pkl", "wb") as f:
            pickle.dump({'odom_gt' : self.odom_gt_msgs, 'odom' : self.odom_msgs}, f)
        print("saved data to ", self.file_location + str(self.testname) + "_f_vel_" + str(F_VEL) + "_a_vel_" + str(A_VEL) + ".pkl")


def main(args=None):
    rclpy.init(args=args)

    odom_test = OdomTest()

    print("starting test")



    while not odom_test.get_reached_speed():
        odom_test.forward_test(F_VEL, A_VEL)
        rclpy.spin_once(odom_test)
        time.sleep(0.1)

    for i in range(10):
        odom_test.forward_test(F_VEL, A_VEL)
        rclpy.spin_once(odom_test)
        time.sleep(0.1)

    print("starting test timer")

    now = time.time()

    while time.time() - now < 10:
        odom_test.forward_test(F_VEL, A_VEL)
        rclpy.spin_once(odom_test)
        time.sleep(0.1)

    odom_test.save_data()

    odom_test.forward_test(0.0, 0.0)

    odom_test.destroy_node()
    rclpy.shutdown()