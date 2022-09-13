from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class SteeringNode(Node):

    def __init__(self):
        super().__init__('wagon_steering_node')

         # robot state
        self.hydrolic_joint = 0.0
        self.wheel_joint = 0.0

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(1)

        # robot state
        self.hydrolic_joint = 0.0
        self.wheel_joint = 0.0
        # wheel_front_right_joint = 0.0
        # wheel_front_left_joint = 0.0
        # wheel_rear_right_joint = 0.0
        # wheel_rear_left_joint = 0.0
        Max = 0
        self.joint_state = JointState()
        self.joint_state.name = ['hydrolic_joint', 
                                    'wheel_front_right_joint', 
                                    'wheel_front_left_joint', 
                                    'wheel_rear_right_joint',
                                    'wheel_rear_left_joint']

        #message declarations
        
        self.base_trans = TransformStamped()
        self.base_trans.header.frame_id = 'world'
        self.base_trans.child_frame_id = 'base_footprint'
    

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)
        self.cmd_vel_sub
        self.get_logger().info("Subscribed to /cmd_vel")

    def loop_around(self, value, limit):
        if value > limit:
            value = -limit
        elif value < -limit:
            value = limit
        return value

    def cmd_vel_callback(self, msg):
        #print("what")
        #self.get_logger().info('I heard: "%s"' % msg.linear.x)
        self.hydrolic_joint = msg.angular.z
        self.wheel_joint += msg.linear.x

        self.loop_around(self.wheel_joint, 2*pi)

        #self.get_logger().info("wheel_joint: {0}".format(self.wheel_joint))
        #self.get_logger().info("hydrolic_joint: {0}".format(self.hydrolic_joint))
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()

        self.joint_state.position = [self.hydrolic_joint,
                                        self.wheel_joint,
                                        self.wheel_joint,
                                        self.wheel_joint,
                                        self.wheel_joint]

        self.joint_pub.publish(self.joint_state)

        self.handle_wagon_pose(msg)
        self.broadcaster.sendTransform(self.base_trans)
    
    def handle_wagon_pose(self, msg):

        # Read message content and assign it to
        # corresponding tf variables
        self.base_trans.header.stamp = self.get_clock().now().to_msg()

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        self.base_trans.transform.translation.x += msg.linear.x
        self.base_trans.transform.translation.y += -sin(msg.angular.z)*0.1
        self.base_trans.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = euler_to_quaternion(0, 0, msg.angular.z)
        self.base_trans.transform.rotation = q
        
        

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = SteeringNode()

    loop_rate = node.create_rate(5)

    try:
        while rclpy.ok():
            rclpy.spin(node)
            #loop_rate.sleep()
    except KeyboardInterrupt:
            pass
    

if __name__ == '__main__':
    main()