# Ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
#Import twist
from geometry_msgs.msg import Twist

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.publisher_ = self.create_publisher(Twist, 'wheel_odom', 10)
        # Declare parameters related to topic names
        self.declare_parameter('joint_state_topic', 'joint_states', 'Joint state topic name')
        self.declare_parameter('wheel_odom_topic', 'wheel_odom', 'Wheel odometry topic name')
        # Get parameters
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        # Create subscriber
        self.subscription = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
        # Create publisher
        self.publisher = self.create_publisher(Twist, self.wheel_odom_topic, 10)

        self.config_index = False
        self.wheel_fl = None
        self.wheel_fr = None
        self.wheel_rl = None
        self.wheel_rr = None
        self.hydraulic_joint = None

        
    def joint_state_callback(self, msg):
        # Get the wheel velocities
        if not self.config_index:
            self.configure_index(msg)
        else:
            # Get the wheel velocities
            wheel_fl_vel = msg.velocity[self.wheel_fl]
            wheel_fr_vel = msg.velocity[self.wheel_fr]
            wheel_rl_vel = msg.velocity[self.wheel_rl]
            wheel_rr_vel = msg.velocity[self.wheel_rr]
            # Get the hydraulic joint position
            hydraulic_joint_pos = msg.position[self.hydraulic_joint]
            # Compute the wheel odometry
            wheel_odom = Twist()
            #
            pass

            # Publish the wheel odometry
            self.publisher.publish(wheel_odom)

        
    def configure_index(self, msg):
        if not self.config_index:
            self.wheel_fl = msg.name.index('wheel_front_left_joint')
            self.wheel_fr = msg.name.index('wheel_front_right_joint')
            self.wheel_rl = msg.name.index('wheel_rear_left_joint')
            self.wheel_rr = msg.name.index('wheel_rear_right_joint')
            self.hydraulic_joint = msg.name.index('hydraulic_joint')

            if self.wheel_fl and self.wheel_fr and self.wheel_rl and self.wheel_rr and self.hydraulic_joint:
                self.config_index = True
                self.get_logger().info('Configuration initialized')
            else:
                self.get_logger().info('Configuration not initialized')

def main():
    pass


if __name__ == '__main__':
    main()
