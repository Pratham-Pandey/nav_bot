import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package_name = "nav_bot"

    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('nav_bot'))
    xacro_file = os.path.join(pkg_path,'description','nav_bot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    
    # Launch Gazebo 
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        # default_value="/home/pratham/ros_development/nav_bot_ws/src/nav_bot/worlds/obstacles_optimized.sdf",
        #default_value="/home/pratham/ros_development/nav_bot_ws/src/nav_bot/worlds/maze2.sdf",
        default_value="/home/pratham/ros_development/nav_bot_ws/src/nav_bot/worlds/maze2_dyn.sdf",
        #default_value="/home/pratham/Desktop/test_world.sdf",
        description='World File to load'
        )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),     # gazebo_ros    gazebo.launch.py
                    launch_arguments={'gz_args': ['-r -v1 ', world], 'on_exit_shutdown': 'true'}.items()
             )
             
    # Spawn Robot
    spawn_bot = Node(package='ros_gz_sim', executable='create',        # executable='spawn_entity.py'
                        arguments=['-topic', 'robot_description',
                                   '-name', 'nav_bot',                  # '-entity', 'nav_bot'],
                                   '-z', '0.1'],               
                        output='screen')

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )
    
    joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state"],
    )
    
    bridge_params= os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')


    # For bridging topics between Gazebo and Ros
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # For image(raw data not info) we need a seprate bridge
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    ros_gz_depth_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/depth/image_raw"]
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', "/home/pratham/ros_development/nav_bot_ws/src/nav_bot/config/rviz_nav_bot_config_2.rviz"],
    )


    # SLAM Localization
    # slam_loc = Node(
    #     package="slam_toolbox",
    #     executable="online_async_launch",
    #     arguments=["/camera/image_raw"]
    # )

    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    online_async_launch = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_async_launch),
        launch_arguments={
            'slam_params_file': '/home/pratham/ros_development/nav_bot_ws/src/nav_bot/config/mapper_params_online_async.yaml',
            'use_sim_time': 'true'
        }.items()
    )

    # Launch Nav2 
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    navigation_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': '/home/pratham/ros_development/nav_bot_ws/src/nav_bot/config/nav2_params.yaml'
        }.items()
    )

    # TwistMux: Remapping Nav2 Output
    twist_mux_params = os.path.join(
        '/home/pratham/ros_development/nav_bot_ws/src/nav_bot/config',
        'twist_mux.yaml'
    )

    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_params],
            remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
        )

    # Dynamic Obstacles: To move objects present in the environment.
    dynamic_obj = Node(
        package="nav_bot",
        executable="dyn_obs",
        name='dyn_obs'
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        world_arg,                      # For World File
        node_robot_state_publisher,
        gazebo,
        spawn_bot,
        diff_drive_controller,
        joint_broadcaster,
        ros_gz_bridge,
        ros_gz_image_bridge,
        ros_gz_depth_image_bridge,
        rviz,
        slam_toolbox_launch,
        nav2_launch,
        twist_mux_node,
        dynamic_obj
    ])
