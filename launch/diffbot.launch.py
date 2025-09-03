import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ================================================================
    # CÁC PHẦN SAU ĐÃ HOẠT ĐỘNG TỐT - GIỮ NGUYÊN
    # ================================================================
    pkg_share = get_package_share_directory('diffbot_sim')
    
    config_file_path = os.path.join(pkg_share, 'config', 'diffbot_params.yaml')

    with open(config_file_path, 'r') as f:
        config_params = yaml.safe_load(f)['robot_params']['ros__parameters']

    robot_params = [f'{key}:={value}' for key, value in config_params.items()]

    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.urdf.xacro'])
    
    xacro_command = ['xacro', ' ', urdf_path]
    for param in robot_params:
        xacro_command.append(' ')
        xacro_command.append(param)
    
    robot_description_content = Command(xacro_command)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 
                     'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'diffbot',
                   '-allow_renaming', 'true']
    )

    # ================================================================
    # TINH CHỈNH Ở ĐÂY: THÊM CẦU NỐI CHO ODOMETRY
    # ================================================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Thêm một chuỗi nữa vào danh sách arguments
        arguments=[
            # Cầu nối 1 (cũ): Từ ROS sang Gazebo cho điều khiển
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            
            # Cầu nối 2 (mới): Từ Gazebo sang ROS cho odometry
            '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
