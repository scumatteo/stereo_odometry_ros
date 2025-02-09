from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os

def generate_launch_description():

    odom_config_file = os.path.join(
        get_package_share_directory('stereo_odometry_ros'),  
        'config',
        'stereo_odometry_params.yaml'
    )
    
    namespaces = [f'stereo_odometry{i+1}' for i in range(1)]
    
    nodes = [
        Node(
            package='stereo_odometry_ros',           
            executable='stereo_odometry_node',      
            namespace=ns,
            name=f'stereo_odometry_node',
            output='screen',
            parameters=[odom_config_file],
            arguments=['--ros-args', '--log-level', 'debug']  
        )
        for ns in namespaces
    ]

    return LaunchDescription(nodes)
