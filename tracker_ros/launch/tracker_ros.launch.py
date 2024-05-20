import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription() 

    outlier_removal=Node(
        package = 'tracker_ros',
        executable = 'outlier_remover'
    )
        
    tracker=Node(
        package = 'tracker_ros',
        executable = 'tracker'
    )
    
    ld.add_action(outlier_removal)
    ld.add_action(tracker)
    return ld
