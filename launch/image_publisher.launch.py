from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os

def generate_launch_description():
    
    image_config_file = os.path.join(
        get_package_share_directory('publisher'),  
        'config',
        'image_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='publisher',           
            executable='image_publisher',      
            name='image_publisher',
            output='screen',
            parameters=[image_config_file]                  
        )
    ])
