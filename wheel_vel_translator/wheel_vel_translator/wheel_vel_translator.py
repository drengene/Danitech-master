
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import time

class WheelVelTranslator(Node):
    def __init__(self):
        super().__init__('wheel_vel_translator')
        # Declare and acquire parameters
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('wheel_radius', 0.292, ParameterDescriptor(description="Wheel radius"))
        self.declare_parameter('front_wheel_separation', 0.963, ParameterDescriptor(description="Separation between front wheels"))
        self.declare_parameter('rear_wheel_separation', 0.963, ParameterDescriptor(description="Separation between rear wheels"))
        self.declare_parameter('front_base_radius', 0.6948+0.085, ParameterDescriptor(description="Length from articulated joint to front wheel axel"))
        self.declare_parameter('rear_base_radius', 0.8716-0.085, ParameterDescriptor(description="Length from articulated joint to rear wheel axel"))

        # Declare topic parameters
        self.declare_parameter('cmd_vel_topic', '/mouse_vel', ParameterDescriptor(description="Topic for receiving velocity commands"))
        self.declare_parameter('joint_state_topic', '/joint_states', ParameterDescriptor(description="Topic for receiving joint states"))
        self.declare_parameter('joint_state_control_topic', '/joint_states_controller', ParameterDescriptor(description="Topic for publishing joint controls"))

        # Get the parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.front_wheel_separation = self.get_parameter('front_wheel_separation').value
        self.rear_wheel_separation = self.get_parameter('rear_wheel_separation').value
        self.front_base_radius = self.get_parameter('front_base_radius').value
        self.rear_base_radius = self.get_parameter('rear_base_radius').value

        # Get the topic parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.joint_state_control_topic = self.get_parameter('joint_state_control_topic').value

        # Create a subscriber to the topic /cmd_vel
        self.subscription = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        
        # Create a subscriber to the topic /joint_states
        self.joint_state_subscription = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)

        # Create a publisher for the topic /joint_states_control. This topic publishes the joint velocities or positions
        self.publisher_ = self.create_publisher(JointState, self.joint_state_control_topic, 10)

        self.alpha = 0.0
        self.alpha_request = 0.0
        self.v = 0.0


    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.alpha_request = msg.angular.z
        self.send_control_vel()

        

    def joint_state_callback(self, msg):
        # Find the joint index for the hydraulic joint
        hydraulic_joint_index = msg.name.index('hydraulic_joint')
        # Get the joint position
        self.alpha = msg.position[hydraulic_joint_index]
        # Calculate the wheel velocities and publish them
        self.send_control_vel()

    def send_control_vel(self):
        v_wheels = self.calc_control_vel()
        joint_state = JointState()
        joint_state.name = ['wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_rear_left_joint', 'wheel_rear_right_joint', 'hydraulic_joint']
        #print(v_wheels)
        joint_state.velocity = [v_wheels[0], v_wheels[1], v_wheels[2], v_wheels[3], 0.0]
        joint_state.position = [0.0, 0.0, 0.0, 0.0, self.alpha_request]
        #print(joint_state)
        self.publisher_.publish(joint_state)


    def calc_control_vel(self):
        # Calculate the wheel velocities
        v_rear = 0
        v_front = 0

        v_wheels = np.zeros(4) # [v_front_left, v_front_right, v_rear_left, v_rear_right]

        if self.alpha == 0:
            v_wheels = np.array([self.v, self.v, self.v, self.v])
            return v_wheels / self.wheel_radius

        if self.rear_base_radius < self.front_base_radius:
            v_rear = self.v
            v_front = self.v * (self.rear_base_radius + self.front_base_radius * np.cos(self.alpha)) / (self.front_base_radius + self.rear_base_radius * np.cos(self.alpha))
        else:
            v_front = self.v
            v_rear = self.v * (self.front_base_radius + self.rear_base_radius * np.cos(self.alpha)) / (self.rear_base_radius + self.front_base_radius * np.cos(self.alpha))

        front_path_radius = (self.rear_base_radius + self.front_base_radius * np.cos(self.alpha)) / np.sin(self.alpha)
        rear_path_radius = (self.front_base_radius + self.rear_base_radius * np.cos(self.alpha)) / np.sin(self.alpha)

        if rear_path_radius == 0:
            #print("Rear path radius is 0")
            relation_front_left = (front_path_radius - self.front_wheel_separation / 2) / front_path_radius
            relation_front_right = (front_path_radius + self.front_wheel_separation / 2) / front_path_radius
            v_wheels = np.array([
                v_front * relation_front_left, 
                v_front * relation_front_right, 
                v_rear * ((-self.rear_wheel_separation / 2) / self.rear_base_radius), 
                v_rear * ((self.rear_wheel_separation / 2) / self.rear_base_radius)
                ])
            return v_wheels / self.wheel_radius
        elif front_path_radius == 0:
            #print("Front path radius is 0")
            relation_rear_left = (rear_path_radius - self.rear_wheel_separation / 2) / rear_path_radius
            relation_rear_right = (rear_path_radius + self.rear_wheel_separation / 2) / rear_path_radius
            v_wheels = np.array([
                v_front * ((-self.front_wheel_separation / 2) / self.front_base_radius), 
                v_front * ((self.front_wheel_separation / 2) / self.front_base_radius), 
                v_rear * relation_rear_left, 
                v_rear * relation_rear_right
                ])
            return v_wheels / self.wheel_radius
        else:
            #print("No path radius is 0")
            #print( "front_path_radius " ,front_path_radius)
            #print( "rear_path_radius " ,rear_path_radius)
            relation_front_left = (front_path_radius - self.front_wheel_separation / 2) / front_path_radius
            #print("( ",front_path_radius, " - ", self.front_wheel_separation, "/ 2 )"," / ", front_path_radius)
            relation_front_right = (front_path_radius + self.front_wheel_separation / 2) / front_path_radius
            relation_rear_left = (rear_path_radius - self.rear_wheel_separation / 2) / rear_path_radius
            relation_rear_right = (rear_path_radius + self.rear_wheel_separation / 2) / rear_path_radius
            #print(self.front_wheel_separation / 2)
            #print(relation_front_left, relation_front_right, relation_rear_left, relation_rear_right)
            v_wheels = np.array([
                v_front * relation_front_left, 
                v_front * relation_front_right, 
                v_rear * relation_rear_left, 
                v_rear * relation_rear_right
                ])
            return v_wheels / self.wheel_radius
    


def main():
    print('Hi from wheel_vel_translator.')
    rclpy.init()
    translator = WheelVelTranslator()
    # Create test values
    translator.v = 0
    # Degrees to radians
    translator.alpha = 0
    # Test the function calc_control_vel
    # Time the function by running it 100000 times with random value of alpha
    t0 = time.time()
    # for i in range(100000):
    #     translator.alpha = np.random.uniform(-np.pi, np.pi)
    #     translator.calc_control_vel()
    # t1 = time.time()
    # print("Time to run 100000 times: ", t1 - t0)
    # # Print the result
    # print("Max frequency: ", 100000 / (t1 - t0))
    res = translator.calc_control_vel()
    # Print all four (rounded to 2 decimals) wheel velocities separately
    print("At alpha = ", translator.alpha, " and v = ", translator.v, " the angular wheel velocities are: ")
    print('Front left wheel velocity: ', round(res[0], 2), "rad/s")
    print('Front right wheel velocity: ', round(res[1], 2), "rad/s")
    print('Rear left wheel velocity: ', round(res[2], 2), "rad/s")
    print('Rear right wheel velocity: ', round(res[3], 2), "rad/s")

    # Loop until the node is killed
    rclpy.spin(translator)
    # Destroy the node explicitly
    translator.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
