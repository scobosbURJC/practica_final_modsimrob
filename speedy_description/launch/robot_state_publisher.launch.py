from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

def generate_launch_description():
    # Declare arguments
    description_file = LaunchConfiguration("description_file", default="speedy.urdf.xacro")
    prefix = LaunchConfiguration("prefix", default="")
    namespace = LaunchConfiguration("namespace", default="")
    use_sim_time = LaunchConfiguration('use_sim_time' , default='true')

    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("speedy_description"), "robots", description_file]),
    ])

    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': robot_description_param,
          'publish_frequency': 100.0,
          'frame_prefix': prefix,
          'namespace': namespace
        }],
    )

    nodes = [
        robot_state_publisher_node
    ]

    return LaunchDescription(nodes)