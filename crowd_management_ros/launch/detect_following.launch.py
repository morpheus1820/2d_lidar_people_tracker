import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
        
    detect_following=Node(
        package = 'crowd_management_ros',
        executable = 'detect_following'
    )

    ld.add_action(detect_following)
    ld.add_action(act_if_not_followed)
    return ld
