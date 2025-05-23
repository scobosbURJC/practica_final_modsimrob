from os.path import join
from os import environ, pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# Function to start the Gazebo server and client
def start_gzserver(context, *args, **kwargs):  
    # TBD: Define pkg_path
    pkg_path = get_package_share_directory('urjc_excavation_world')

    world_name = LaunchConfiguration('world_name').perform(context)
    # TBD: Define world path
    world = join(pkg_path, 'worlds', world_name + '.world')

    # Launch Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v 4 ', world]}.items()
    )

    # Launch Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
    )

    return [start_gazebo_server_cmd, start_gazebo_client_cmd]

# Function to get the model paths
def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = join(package_path, "share")

        model_paths += model_path

    if 'GZ_SIM_RESOURCE_PATH' in environ:
        model_paths += pathsep + environ['GZ_SIM_RESOURCE_PATH']

    return model_paths

# Launch description
def generate_launch_description():
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description="use_sim_time simulation parameter"
    )
    model_path = ''
    resource_path = ''

    pkg_path = get_package_share_directory('speedy_description')
    model_path += join(pkg_path, 'models')
    resource_path += pkg_path + model_path

    if 'GZ_SIM_MODEL_PATH' in environ:
        model_path += pathsep+environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in environ:
        resource_path += pathsep+environ['GZ_SIM_RESOURCE_PATH']

    model_path = get_model_paths(['speedy_description'])

    speedy_description_launcher = IncludeLaunchDescription(
       PathJoinSubstitution(
           [FindPackageShare("speedy_moveit_config"), "launch", "rsp.launch.py"]
       ),
    )

    start_gazebo_server_cmd = OpaqueFunction(function=start_gzserver)

    # TBD: RViz config and launching
    moveit_config_path = join(get_package_share_directory("speedy_moveit_config"), "config")

    pilz_cartesian_limits_file_path = join(moveit_config_path, 'pilz_cartesian_limits.yaml')
    moveit_config = (MoveItConfigsBuilder("speedy", package_name="speedy_moveit_config")
                        .planning_pipelines(
                            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                            default_planning_pipeline="ompl"
                        )
                        .planning_scene_monitor(
                            publish_robot_description=False,
                            publish_robot_description_semantic=True,
                            publish_planning_scene=True,
                        )
                    ).to_moveit_configs()

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[
            #moveit_config.planning_pipelines,
            {
                'config_file': join(
                    pkg_path, 'rviz', 'robot.rviz'
                ),
                'use_sim_time': True,
            }
        ]
    )

    # TBD: robot spawner
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'speedy',
            '-topic', 'robot_description',
            '-use_sim_time', 'True',
            '-x', '0',
            '-y', '0',
            '-z', '1.8'
        ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': join(
                    pkg_path, 'config', 'speedy_bridge.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # Image bridge
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/front_camera/image",
            "/gripper_camera/image"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True,
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Twist stamped
    twist_stamped = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }],
        remappings={('cmd_vel_out', '/speedy_base_control/cmd_vel'),
                    ('cmd_vel_in', '/cmd_vel')},
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path))
    ld.add_action(speedy_description_launcher)
    ld.add_action(declare_sim_time)
    ld.add_action(bridge)
    ld.add_action(gz_image_bridge_node)
    ld.add_action(start_gazebo_server_cmd)
    # TBD: RViz config and launching
    ld.add_action(rviz_cmd)
    # TBD: robot spawner 
    ld.add_action(gazebo_spawn_robot)
    ld.add_action(twist_stamped)

    return ld
