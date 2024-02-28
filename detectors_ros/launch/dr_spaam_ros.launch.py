import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ld = LaunchDescription()
    
    ckpt_path = os.path.join(get_package_share_directory('detectors_ros'), 'checkpoints', 'ckpt_jrdb_ann_dr_spaam_e20.pth')
    
    config = os.path.join(
        get_package_share_directory('detectors_ros'),
        'config',
        'dr_spaam_ros.yaml'
        )
		
    param_substitutions = {
        'weight_file': ckpt_path}
	
    configured_params = RewrittenYaml(
        source_file=config,
        param_rewrites=param_substitutions,
        convert_types=True)

       
    node=Node(
        package = 'detectors_ros',
        executable = 'dr_spaam_ros',
        parameters = [configured_params]
    )
    ld.add_action(node)
    return ld
