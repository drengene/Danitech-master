
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class WheelVelTranslator(Node):
    def __init__(self):
        super().__init__('wheel_vel_translator')
        # Declare and acquire parameters
        self.declare_parameter('wheel_radius', 0.1, "Wheel radius")
        self.declare_parameter('front_wheel_separation', 0.5, "Separation between front wheels")
        self.declare_parameter('rear_wheel_separation', 0.5, "Separation between rear wheels")
        self.declare_parameter('front_base_radius', 0.6, "Length from articulated joint to front wheel axel")
        self.declare_parameter('rear_base_radius', 0.6, "Length from articulated joint to rear axel")

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.front_wheel_separation = self.get_parameter('front_wheel_separation').value
        self.rear_wheel_separation = self.get_parameter('rear_wheel_separation').value
        self.front_base_radius = self.get_parameter('front_base_radius').value
        self.rear_base_radius = self.get_parameter('rear_base_radius').value

        # Create a subscriber to the topic /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Create a subscriber to the topic /joint_states
        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        # Create a publisher for the topic /wheel_vel

        self.alpha = None
        self.v = None


    def cmd_vel_callback(self, msg):
        self.v_front = msg.linear.x
        self.alpha = msg.angular.z

    def joint_state_callback(self, msg):
        # Get the joint positions
        # Calculate the base velocity
        # Publish the base velocity
        pass

    def calc_control_vel(self):
        # First we calculate the relationship between linear front and rear center velocity
        # Then we calculate the linear front and rear wheel velocities
        # Finally we calculate the wheel velocities
        relationship = (self.front_base_radius + self.rear_base_radius * np.cos(self.alpha) ) / (self.rear_base_radius + self.front_base_radius * np.cos(self.alpha))
        v_rear = relationship * self.v_front
        v_front_left = self.v_front - self.front_wheel_separation * self.alpha / 2
        v_front_right = self.v_front + self.front_wheel_separation * self.alpha / 2
        v_rear_left = v_rear - self.rear_wheel_separation * self.alpha / 2
        v_rear_right = v_rear + self.rear_wheel_separation * self.alpha / 2
    


def main():
    print('Hi from wheel_vel_translator.')


if __name__ == '__main__':
    main()
