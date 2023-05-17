# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from logging import root
import sys
#from asyncio.timeouts import timeout

from omni.isaac.kit import SimulationApp

FPS_RATE = 1000
headless_streaming = False

if len(sys.argv) > 1 :
    if "stream" in sys.argv:
        headless_streaming = True
        print("Starting isaac sim in headless with streaming")

    else:
        print("invalid arguments parsed, use:")
        print("stream, true or yes")
        print("to start in headless mode with streaming")

    # Check if any string in entire list argv is FPS
    if "FPS" in sys.argv or "fps" in sys.argv:      # Is actually step pr second, not frame pr second...
        # get index of FPS in list
        index = sys.argv.index("FPS") if "FPS" in sys.argv else sys.argv.index("fps")
        # get FPS value
        FPS_RATE = int(sys.argv[index + 1]) if int(sys.argv[index + 1]) else FPS_RATE
        print("FPS set to: ", FPS_RATE)
    else:
        print("FPS not specified, using default value of 1000")
    

simulation_app = SimulationApp({"headless": headless_streaming})  # we can also run as headless.     
# ! MUST BE DONE BEFORE IMPORTING ANYTHING ELSE !
# ^ HAS TO BE HERE ^


# Omni imports
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
import omni.kit.commands
from omni.isaac.core.utils import stage
from omni.isaac.core.prims import XFormPrim
import omni.kit.app
from omni.usd import get_context
# Import physicsmaterials
from omni.isaac.core.materials.physics_material import PhysicsMaterial

#from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools
from pxr import Sdf, UsdLux
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import GeometryPrimView
from omni.isaac.articulation_inspector import *

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.articulation_inspector")


if headless_streaming:
    # Default Livestream settings
    simulation_app.set_setting("/app/window/drawMouse", True)
    simulation_app.set_setting("/app/livestream/proto", "ws")
    simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
    simulation_app.set_setting("/ngx/enabled", False)

    # Note: Only one livestream extension can be enabled at a time
    # Enable Native Livestream extension
    # Default App: Streaming Client from the Omniverse Launcher
    enable_extension("omni.kit.livestream.native")



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Other imports
import numpy as np
import time
import os
import asyncio

from include.Sensors.rtx_lidar import lidar_3d
from include.Sensors.imu import IMU
from include.Sensors.lidar_3d import lidar
from include.TF.joint_state_publisher import joint_state_pub
from include.Sensors.gps import gps_pub
from include.Sensors.pose import pose_pub
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
# To run file in terminal, use:
# ~/.local/share/ov/pkg/isaac_sim-2022.1.1/python.sh wagon_in_scene.py

# Create class to hold the simulation app



class IsaacSim(Node):
    def __init__(self):
        super().__init__('isaac_sim')
        self.world = World(physics_dt=1/60)
        self.timeline = omni.timeline.get_timeline_interface()
        self.pause_world()

        #self.world.scene.add_default_ground_plane()
        #self.world.reset()

        self.stage = omni.usd.get_context().get_stage()

        self.set_physics()

        self.import_usd() # Import the world USD 
        self.world.scene.add_default_ground_plane()

        self.world.reset()
        self.import_from_urdf()


        print( "This robot valid innit:" ,self.robot.is_valid())
        self.setup_ros_subscribers()

        self.i = 0
   

    def setup_ros_subscribers(self):

        # self.twist_sub = self.create_subscription(
        #     Twist,
        #     'cmd_vel',
        #     self.cmd_vel_callback,
        #     1)
        # print("Subscribed to cmd_vel")
        #self.twist_sub  # prevent unused variable warning

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states_controller',
            self.joint_state_controller_callback,
            1)
        print("Subscribed to joint_states_controller")
        self.joint_state_sub  # prevent unused variable warning

        self.position_reset_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'set_pose',
            1)

        # Publish initial pose
        init_pose  = PoseWithCovarianceStamped()
        init_pose.header.stamp.sec = 1
        init_pose.header.stamp.nanosec = 1
        init_pose.header.frame_id = "odom"
        self.position_reset_pub.publish(init_pose)
        print("Publishing to initialpose")

        # Publish simulation time
        self.sim_time_pub = self.create_publisher(
            Clock,
            'clock',
            1)
        print("Publishing to clock")

    
    def pub_sim_time(self):
        # Get current simulation time
        sim_time = self.world.current_time
        sim_time_msg = Clock()
        sim_time_msg.clock.sec = int(sim_time)
        sim_time_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self.sim_time_pub.publish(sim_time_msg)



    def set_physics(self):

        # Add lighting
        distantLight = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)


    def cmd_vel_callback(self, msg):

        #self.get_logger().info('I heard: "%s"' % msg)

        self.robot.apply_wheel_actions(
            ArticulationAction(joint_positions=None, joint_efforts=None, 
                joint_velocities=[0,0, msg.linear.x, 0], joint_indices=[2,3,4,5])
        )
        self.robot.apply_action(
            ArticulationAction(joint_positions=[msg.angular.z], joint_indices=[1])
        )
        

    def import_usd(self):
        #path = "/home/danitech/isaac_ws/environments/USD/grass_terrain.usd"
        #path = "/home/danitech/isaac_ws/environments/USD/Quarray_en_cantera.usd"
        #path = "/home/danitech/isaac_ws/environments/USD/fun_in_a_bun.usd"
        path = "/home/danitech/isaac_ws/environments/USD/simple_ramp.usd"
        #path = "/home/danitech/Documents/mesh_world.usd"
        #path = "/home/danitech/isaac_ws/environments/USD/cube_box_world.usd"

        #path = "/home/danitech/master_ws/src/Danitech-master/wagon_isaac/usd/environments/warehouse.usd"    
        
        prim_path="/World"
        prim_stage = stage.add_reference_to_stage(usd_path=path, prim_path=prim_path)
        prim = XFormPrim(
            prim_path=prim_path, name="grass",
            position=np.array([0, 0, 0]), # Position for wagon in center of quarry scene
            #position=np.array([0, 0.0, 0.0]),
            #orientation=np.array([-0.7071068, 0, 0, 0.7071068])
        )
        self.world.scene.add(prim)

        self.world.reset()

    def joint_state_controller_callback(self, msg):
        #print("Callback")
        #self.get_logger().info('I heard on joint controller: "%s"' % msg)
        wheel_joint_indices = [msg.name.index("wheel_front_left_joint"), msg.name.index("wheel_front_right_joint"), msg.name.index("wheel_rear_left_joint"), msg.name.index("wheel_rear_right_joint")]
        wheel_velocities = [msg.velocity[wheel_joint_indices[0]], msg.velocity[wheel_joint_indices[1]], msg.velocity[wheel_joint_indices[2]], msg.velocity[wheel_joint_indices[3]]]
        self.robot.apply_action(
            ArticulationAction(joint_velocities=wheel_velocities, joint_indices=[2,3,4,5])
        )

        hydraulic_joint_index = msg.name.index("hydraulic_joint")
        hydraulic_position = msg.position[hydraulic_joint_index]
        self.robot.apply_action(
            ArticulationAction(joint_positions=[hydraulic_position], joint_indices=[1])
        )

        # Print with nice formatting
        # print("Joint States:")
        # for i in range(len(msg.name)):
        #     print("  ", msg.name[i], ":", msg.position[i], ":", msg.velocity[i])


    def import_from_urdf(self):
        # setting up import configuration: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.urdf/docs/index.html#omni.isaac.urdf._urdf.ImportConfig
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.set_merge_fixed_joints(False)
        import_config.set_convex_decomp(False)
        import_config.set_import_inertia_tensor(True)
        import_config.set_fix_base(False)
        import_config.set_default_drive_type(2) # 2 for velocity 
        import_config.set_default_drive_strength(100.00)
        import_config.set_default_position_drive_damping(15000.00)

        # import URDF
        omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path="/home/danitech/master_ws/src/Danitech-master/wagon_description/urdf/wagon.urdf",
            import_config=import_config,
        )
        # import USD

        self.wagon_prim_path = "/wagon"

        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path=self.wagon_prim_path,
                # position=np.array([-40.5, -199, 59.58]),
                position=np.array([0, 0, 0.5]),
                name="wagon",
                wheel_dof_names=["wheel_front_left_joint", "wheel_front_right_joint", "wheel_rear_left_joint", "wheel_rear_right_joint"],
                wheel_dof_indices=[3,2,5,4],
                create_robot=True,
            )
        )

        # Create physics material for robot wheels
        wheel_material = PhysicsMaterial(
            name="wheel_material",
            static_friction=7,
            dynamic_friction=2.5,
            restitution=0.1,
            prim_path= self.wagon_prim_path + "/wheels_material"
        )
        #self.world.scene.add(wheel_material)

        self.wheel_prim_view = GeometryPrimView(self.wagon_prim_path + "/wheel.*link/collisions")

        print("GEom prim paths; ", self.wheel_prim_view.prim_paths)
        self.wheel_prim_view.apply_physics_materials(wheel_material)

        # Assign material to robot wheels
        

        self.world.step()
        self.robot.initialize()

        self.robot_view = ArticulationView(prim_paths_expr=self.wagon_prim_path, name="wagon_view")
        self.world.scene.add(self.robot_view)
        self.robot_view.initialize()
        self.world.reset_async()




    
    def set_params(self):
        # Set the parameters of the hydraulic joint
        stage = get_context().get_stage()
        prim_path = self.wagon_prim_path + "/connection_link/hydraulic_joint"
        prim=stage.GetPrimAtPath(prim_path)
        print("Prim: ", prim)
        hydraulic_s = prim.GetAttribute("drive:angular:physics:stiffness")
        hydraulic_d = prim.GetAttribute("drive:angular:physics:damping")
        hydraulic_force = prim.GetAttribute("drive:angular:physics:maxForce")
        hydraulic_s.Set(10000.00)
        hydraulic_d.Set(150.00)
        hydraulic_force.Set(1500.0)
        #print("max efforts hydrolic:", self.robot_view.get_max_efforts(joint_indices=[1]))
        #self.robot_view.set_max_efforts([500.0], joint_indices=[1])
        #print("max efforts hydrolic:", self.robot_view.get_max_efforts(joint_indices=[1]))

        print("Stiffness: ", hydraulic_s.Get())
        print("Damping: ", hydraulic_d.Get())
        print("Max Force: ", hydraulic_force.Get())

        # Set the parameters of the connection joint
        prim_path = self.wagon_prim_path + "/base_link/connection_joint"
        prim=stage.GetPrimAtPath(prim_path)
        connection_s = prim.GetAttribute("drive:angular:physics:stiffness")
        connection_d = prim.GetAttribute("drive:angular:physics:damping")
        connection_force = prim.GetAttribute("drive:angular:physics:maxForce")
        connection_s.Set(0.00)
        connection_d.Set(10.00)
        connection_force.Set(0.00)


        
        prim_path  = [self.wagon_prim_path + "/rear_link/wheel_rear_left_joint", self.wagon_prim_path + "/rear_link/wheel_rear_right_joint",
             self.wagon_prim_path + "/base_link/wheel_front_left_joint", self.wagon_prim_path + "/base_link/wheel_front_right_joint"]
        for prim in prim_path:
            prim=stage.GetPrimAtPath(prim)
            wheel_s = prim.GetAttribute("drive:angular:physics:stiffness")
            wheel_d = prim.GetAttribute("drive:angular:physics:damping")
            wheel_f = prim.GetAttribute("drive:angular:physics:maxForce")
            wheel_armature = prim.GetAttribute("physxJoint:armature")
            wheel_s.Set(0.00)
            wheel_d.Set(4500.00)
            wheel_f.Set(40000.00)
            wheel_armature.Set(4000.00)

        print("Armatures: ", self.robot_view.get_armatures(joint_indices=[2,3,4,5]))
        #self.robot_view.set_friction_coefficients([2000.0, 2000.0, 2000.0, 2000.0], indices=)
        print("friction: ", self.robot_view.get_friction_coefficients(joint_indices=[2,3,4,5]))


    def pause_world(self):
        self.timeline.pause()
        self.world.pause()

    def play_world(self):
        self.timeline.play()
        self.world.play()

    def step(self):
        self.world.step(render=True)


def main():
    rclpy.init()
    isaac_sim = IsaacSim()

    for i in range(10):
        isaac_sim.step()

    isaac_sim.set_params()

    #lidar = lidar_3d("/lidar", isaac_sim.wagon_prim_path + "/base_scan", "Example_Rotary")
    lidar_sim = lidar("/lidar", isaac_sim.wagon_prim_path + "/base_scan")
    time.sleep(1)

    imu_scan = IMU( isaac_sim.wagon_prim_path + "/base_scan_imu_link", False,  frame_id="base_scan_imu_link")
    imu_rear =  IMU( isaac_sim.wagon_prim_path + "/rear_imu_link", True,frame_id="rear_imu_link")

    simulation_app.update()

    last_spin = time.time()
    time_now = time.time()

    joint_states = joint_state_pub("wagon")
    gps_module = gps_pub( isaac_sim.wagon_prim_path + "/rtk_pole", init_lat=55.471650, init_lon=10.328990)
    pose_publisher = pose_pub(isaac_sim.wagon_prim_path + "/base_link", "world")
    rear_pose_publisher = pose_pub(isaac_sim.wagon_prim_path + "/rear_link", "world", "rear_link")
    i = 0

    isaac_sim.play_world()

    #isaac_sim.pause_world()

    while True:
        #if True:
        if time.time() - last_spin > 1/FPS_RATE: # 60 Hz

            last_spin = time.time()
            isaac_sim.step()
            isaac_sim.pub_sim_time()
            rclpy.spin_once(isaac_sim, timeout_sec=0.005)
            
            imu_scan.ros_pub()
            imu_rear.ros_pub()


            #print("time: ", time.time() - time_now)
            
            #if i > 300:

            if i % 6 == 0:
                gps_module.pub_gps_data()
                lidar_sim.ros_pub()
                
            pose_publisher.ros_pub()
            rear_pose_publisher.ros_pub()


            joint_states.pub()


            if i % 600 == 0:

                print("Heartbeat: ", i/60)
                #Print current physics time
                #print("Physics t: ", isaac_sim.world.current_time)
                #Print many seconds pr second (should be close to 1)
                print("seconds pr second: ", 1/((time.time()-time_now) * 60))

            time_now = time.time()

            i+=1




    simulation_app.close()  # close Isaac Sim

if __name__ == "__main__":
    main()
