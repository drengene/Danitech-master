from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	# Declare the launch arguments
	bag_file_arg = '/home/junge/Documents/bags/montecarlo_dataset/rosbag2_2023_03_31-14_35_32'

	# Play the rosbag
	rosbag_player = ExecuteProcess(
		cmd=['ros2', 'bag', 'play', bag_file_arg],
		name='rosbag_player'
	)
	
	world_tf_pub = Node(
		package='wagon_navigation',
		executable='world_tf_pub',
		name='world_tf_pub',
		output='screen'
	)

	# Launch the node
	node = Node(
		package='lmao',
		executable='Localizer',
		name='localizer',
		output='screen'        
	)
	

	# Include the launch description
	return LaunchDescription([
		rosbag_player,
		world_tf_pub,
		node
	])
