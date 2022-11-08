# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from logging import root
import sys
#from asyncio.timeouts import timeout

from omni.isaac.kit import SimulationApp
if len(sys.argv) > 1 :
    if str(sys.argv[0]) == "stream" or "Stream" or "True" or "yes":
        headless_streaming = True
        print("Starting isaac sim in headless with streaming")
    else:
        print("invalid arguments parsed, use:")
        print("stream, true or yes")
        print("to start in headless mode with streaming")
else:
    headless_streaming = False

simulation_app = SimulationApp({"headless": headless_streaming})  # we can also run as headless.     
# ! MUST BE DONE BEFORE IMPORTING ANYTHING ELSE !
# ^ HAS TO BE HERE ^


# Omni imports
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
import omni.kit.commands
from omni.isaac.core.utils import stage, extensions, nucleus, prims, xforms
from omni.isaac.core.prims import XFormPrim
import omni.kit.app
from omni.usd import get_context

#from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools
from pxr import Sdf, Gf, UsdPhysics, UsdLux, PhysxSchema, UsdGeom
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.isaac_sensor import _isaac_sensor


from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

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
import carb

from include.Sensors.rtx_lidar import lidar_3d
from include.Sensors.imu import IMU
from include.Sensors.lidar_3d import lidar
from include.TF.joint_state_publisher import joint_state_pub
from include.Sensors.gps import gps_pub
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
        #self.import_wagon()
        self.stage = omni.usd.get_context().get_stage()

        self.set_physics()

        self.import_from_urdf()
        self.import_usd()
        self.world.reset()
        print( "This robot valid innit:" ,self.robot.is_valid())

 

        self.i = 0
        self.twist_sub = self.create_subscription(
            Twist,
            'mouse_vel',
            self.cmd_vel_callback,
            1)
        print("Subscribed to cmd_vel")
        self.twist_sub  # prevent unused variable warning
        


    def set_physics(self):


        # scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene"))
        # # Set gravity
        # scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        # scene.CreateGravityMagnitudeAttr().Set(9.81)
        # # Set solver settings
        # PhysxSchema.PhysxSceneAPI.Apply(self.stage.GetPrimAtPath("/physicsScene"))
        # physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(self.stage, "/physicsScene")
        # physxSceneAPI.CreateEnableCCDAttr(True)
        # physxSceneAPI.CreateEnableStabilizationAttr(True)
        # physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        # physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        # physxSceneAPI.CreateSolverTypeAttr("TGS")

        # Add lighting
        distantLight = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)




    def cmd_vel_callback(self, msg):
        #print("Callback")
        #print("Callback heard", msg)
        #self.get_logger().info('I heard: "%s"' % msg)
        self.robot.apply_action(
            ArticulationAction(joint_velocities=[msg.linear.x, msg.linear.x, msg.linear.x, msg.linear.x], joint_indices=[2,3,4,5])
        )
        self.robot.apply_action(
            ArticulationAction(joint_positions=[msg.angular.z], joint_indices=[1])
        )

    def import_usd(self):
        path = "/home/danitech/isaac_ws/environments/grass_terrain.usd"
        prim_path="/World"
        stage.add_reference_to_stage(usd_path=path, prim_path=prim_path)
        prim = XFormPrim(
            prim_path=prim_path, name="grass",
            position=np.array([0, 0, 1.5]),
            #orientation=np.array([-0.7071068, 0, 0, 0.7071068])
        )
        self.world.scene.add(prim)

        self.world.reset()







    def import_from_urdf(self):
        # setting up import configuration: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.urdf/docs/index.html#omni.isaac.urdf._urdf.ImportConfig
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.set_merge_fixed_joints(False)
        import_config.set_convex_decomp(False)
        import_config.set_import_inertia_tensor(True)
        import_config.set_fix_base(False)
        import_config.set_default_drive_type(1) # velocity 
        import_config.set_default_drive_strength(2000.00)
        import_config.set_default_position_drive_damping(100.00)

        # import URDF
        omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path="/home/danitech/master_ws/src/Danitech-master/wagon_description/urdf/wagon.urdf",
            import_config=import_config,
        )
        # import USD

        # self.controller = add_reference_to_stage(usd_path="/home/danitech/isaac_ws/Controllers/wagon_controller_keyboard.usd", prim_path="/World/ControllerGraph", prim_type="Xform")

        self.world.scene.add(
            WheeledRobot(
                prim_path="/wagon",
                name="wagon",
                wheel_dof_names=["wheel_front_right_joint", "wheel_front_left_joint", "wheel_rear_right_joint", "wheel_rear_left_joint"],
                wheel_dof_indices=[2,3,4,5],
                create_robot=True,
            )
        )

        #prim = define_prim("/World/Controller", "ComputeGraph")
        #prim.GetReferences().AddReference("/home/danitech/Documents/WagonController.usd")
        
        self.robot = self.world.scene.get_object("wagon")

    
    def set_params(self):
        stage = get_context().get_stage()
        prim_path ="/wagon/connection_link/hydraulic_joint"

        prim=stage.GetPrimAtPath(prim_path)
        hydraulic_s = prim.GetAttribute("drive:angular:physics:stiffness")
        hydraulic_d = prim.GetAttribute("drive:angular:physics:damping")
        hydraulic_force = prim.GetAttribute("drive:angular:physics:maxForce")
        hydraulic_s.Set(1000.00)
        hydraulic_d.Set(150.00)
        hydraulic_force.Set(5000.0)

        print("Stiffness: ", hydraulic_s.Get())
        print("Damping: ", hydraulic_d.Get())
        print("Max Force: ", hydraulic_force.Get())

    def move_robot(self):

        self.robot.apply_action(
            ArticulationAction(joint_velocities=[1, 1, 1, 1], joint_indices=[2,3,4,5])
        )
        self.robot.apply_action(
            ArticulationAction(joint_positions=[0.4], joint_indices=[1])
        )


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
    # Print info about the jetbot after the first reset is called
    #print("Num of degrees of freedom after first reset: " + str(_jetbot.num_dof)) # prints 2
    #print("Joint Positions after first reset: " + str(_jetbot.get_joint_positions()))
    # Step the simulation in a loop running real time for 500 steps
    

    for i in range(10):
        isaac_sim.step()



    isaac_sim.set_params()

    print("Crash before lidar")
    #lidar = lidar_3d("/lidar", "/wagon/base_scan", "Example_Rotary")
    lidar_sim = lidar("/lidar", "/wagon/base_scan")
    time.sleep(1)
    print("Crash after lidar")

    imu = IMU("/wagon/base_scan", False, "sensor")
    simulation_app.update()
    simulation_app.update()
    print("Crash after reset")

    last_spin = time.time()
    # lidarInterface = _range_sensor.acquire_lidar_sensor_interface() # Used to interact with the LIDAR
    joint_states = joint_state_pub("wagon")
    gps_module = gps_pub("/wagon/rtk_pole")
    i = 0
    #imu = _isaac_sensor.acquire_imu_sensor_interface()
    
    isaac_sim.play_world()
    while True:
        if True:
        #if time.time() - last_spin > 1/60:
            last_spin = time.time()
            isaac_sim.step()
            rclpy.spin_once(isaac_sim, timeout_sec=0.005)
            imu.ros_pub()
            #time_now = time.time()
            lidar_sim.ros_pub()
            #print("time: ", time.time() - time_now)
            
            
            #if i > 300:
            if i % 15 == 0:
                gps_module.pub_gps_data()
                joint_states.joint_state_callback()

            


            i+=1
        

            # if i == 50:
            #     _imu_sensor = _isaac_sensor.acquire_imu_sensor_interface()
            #     imu = True

            if i % 600 == 0:

                print("Heartbeat: ", i/60)
                #Print current physics time
                #print("Physics t: ", isaac_sim.world.current_time)
                #print(_imu_sensor.is_imu_sensor("wagon/base_scan/sensor"))
                #sensor_reading = imu.get_sensor_readings("/wagon/base_scan/sensor")
                #print("Imu data: ", sensor_reading)
                
                #Print many seconds pr second (should be close to 1)
                print("seconds pr second: ", 1/((time.time()-last_spin) * 60))



    simulation_app.close()  # close Isaac Sim

if __name__ == "__main__":
    main()
