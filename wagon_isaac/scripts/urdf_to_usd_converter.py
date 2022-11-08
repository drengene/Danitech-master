
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})  # we can also run as headless.     

import omni.kit.commands
from omni.isaac.urdf import _urdf


def import_from_urdf():
        # setting up import configuration: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.urdf/docs/index.html#omni.isaac.urdf._urdf.ImportConfig
        #status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        urdf_interface = _urdf.acquire_urdf_interface()
        import_config = _urdf.ImportConfig()
        import_config.set_merge_fixed_joints(False)
        import_config.set_convex_decomp(False)
        import_config.set_import_inertia_tensor(False)
        import_config.set_fix_base(False)
        import_config.set_default_drive_type(2) # velocity 
        import_config.set_default_drive_strength(10000000.00)
        import_config.set_default_position_drive_damping(100000.00)
        import_config.create_physics_scene = False
        # import URDF
        result = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path="/home/danitech/master_ws/src/Danitech-master/wagon_description/urdf/wagon.urdf",
            import_config=import_config,
            dest_path="/home/danitech/master_ws/src/Danitech-master/wagon_description/usd/wagon_autogen.usd"
        )

        # Change stiffness of hydraulic joint

        
        return result

print(import_from_urdf())