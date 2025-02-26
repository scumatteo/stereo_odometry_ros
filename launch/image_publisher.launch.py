from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os

def generate_launch_description():
    
    image_config_file = os.path.join(
        get_package_share_directory('stereo_odometry_ros'),  
        'config',
        'image_publisher_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='stereo_odometry_ros',           
            executable='image_publisher_node',      
            name='image_publisher_node',
            output='screen',
            parameters=[image_config_file]                  
        )
    ])
