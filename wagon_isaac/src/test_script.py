from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
from omni.usd import get_context

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.setup_scene()  
        return

    def setup_scene(self):
        world = World.instance()
        #world.scene.add_default_ground_plane()
        stage = get_context.get_stage()
        print(stage)

        return

# import omni

# stage = omni.usd.get_context().get_stage()

# wagon = stage.GetPrimAtPath("/wagon")

# joints = []

# for child in wagon.GetChildren():
# 	for child2 in child.GetChildren():
# 		print(child.GetTypeName())
# #		if child.GetTypeName() = "PhysicalRevoluteJoint":
# 	#		joints.append(child)
# print(joints)


# import omni

# stage = omni.usd.get_context().get_stage()

# rtk = stage.GetPrimAtPath("/wagon/rtk_pole")

# print(rtk.GetAttribute("xformOp:translate").Get()[0])