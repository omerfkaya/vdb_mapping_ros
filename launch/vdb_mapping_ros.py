import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('vdb_mapping_ros'),
        'config',
        'vdb_params.yaml'
        )
        
    node=Node(
        package = 'vdb_mapping_ros',
        name = 'vdb_mapping_ros_node',
        executable = 'vdb_mapping_ros_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld