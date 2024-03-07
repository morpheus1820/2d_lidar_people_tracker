import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
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
        convert_types=True
        )
        
    node=LifecycleNode(
        package = 'detectors_ros',
        name = 'dr_spaam_ros',
        executable = 'dr_spaam_ros',
        parameters = [configured_params],
        output='screen',
        namespace=''
        )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate node
    emit_event_to_request_active_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )    
    
    ld.add_action(node)
    ld.add_action(emit_event_to_request_configure_transition)
    ld.add_action(emit_event_to_request_active_transition)
    return ld
