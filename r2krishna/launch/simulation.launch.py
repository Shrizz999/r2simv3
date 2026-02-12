import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    pkg_r2 = get_package_share_directory('r2krishna')
    pkg_arena = get_package_share_directory('arena_viz')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    r2_parent = str(Path(pkg_r2).parent.resolve())
    arena_parent = str(Path(pkg_arena).parent.resolve())
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[r2_parent, ':', arena_parent, ':', pkg_arena]
    )

    world_file = os.path.join(pkg_arena, 'worlds', 'arena.world')
    urdf_file = os.path.join(pkg_r2, 'urdf', 'r2krishna.urdf')
    rviz_config = os.path.join(pkg_r2, 'rviz', 'default.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        gz_resource_path,

        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # 2. Gazebo Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': f'-r {world_file}'}.items()
        ),

        # 3. Spawn Robot
        Node(
            package='ros_gz_sim', 
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'r2krishna', '-x', '1.0', '-y', '1.0', '-z', '0.5']
        ),

        # 4. ROS-Gazebo Bridge (INCLUDES ULTRASONIC)
        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                # Commands
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/cmd_vel_front@geometry_msgs/msg/Twist]gz.msgs.Twist',
                
                # Odometry & TF
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                
                # Sensors
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/bumper_states@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                
                # *** NEW: Ultrasonic Bridge ***
                '/ultrasonic@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' 
            ]
        ),

        # 5. Static Transforms (INCLUDES ULTRASONIC TF)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'r2krishna/base_footprint/lidar'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'r2krishna/base_footprint/camera'],
            parameters=[{'use_sim_time': True}]
        ),
        # *** NEW: Explicit TF for Ultrasonic (Matches URDF Position) ***
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            # x=0.32, z=0.05 match your URDF joint
            arguments=['0.32', '0', '0.05', '0', '0', '0', 'base_link', 'ultrasonic_link'],
            parameters=[{'use_sim_time': True}]
        ),

        # 6. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_slam, 'launch', 'online_async_launch.py')]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # 7. RViz
        TimerAction(
            period=5.0, 
            actions=[Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}])]
        )
    ])
