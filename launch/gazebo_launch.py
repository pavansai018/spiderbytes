import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'spiderbytes'), 'urdf', 'spider.xacro')
    control_yaml_file = os.path.join(get_package_share_directory('spiderbytes'), 'config', 'ros2_control.yaml')
    # world = LaunchConfiguration('world')

    robot_desc = ParameterValue(Command(['xacro ', urdf, ' ', 'ros2_control_yaml:=', control_yaml_file]),
                                       value_type=str)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file to use in Gazebo')
    
    gz_world_arg = LaunchConfiguration('world')

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : '-r empty.sdf', #gz_world_arg 
        }.items()
    )
    
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "spider_v3",
            "-allow_renaming", "true",
            "-z", "0.1",
        ]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # "/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            # "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            # '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
    )

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])
    # joint_state_publisher_gui: publishes /joint_states with sliders
    jsp_gui_params = {
        'use_sim_time': use_sim_time,
        # 'robot_description': robot_desc
    }

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[jsp_gui_params],
        remappings=[('/joint_states', '/gui_joint_states')]
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )   

    spawner_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    jsp_to_traj_node = Node(
        package='spiderbytes',
        executable='jsp_to_traj',
        output='screen',
    )
    spawn_controllers_after_entity = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[spawner_jsb, spawner_jtc],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    # ld.add_action(spawner_jsb)
    # ld.add_action(spawner_jtc)
    ld.add_action(spawn_controllers_after_entity)
    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    # Launch Joint State Publisher GUI
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(jsp_to_traj_node)
    return ld