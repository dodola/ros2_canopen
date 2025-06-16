# canopen_402_driver_ros2/launch/cia402_node.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node # Used for non-lifecycle nodes, LifecycleNode for lifecycle nodes
from launch_ros.actions import LifecycleNode # Correct import for LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import launch.events # Required for launch.events.Shutdown

def generate_launch_description():
    # --- Declare Launch Arguments ---
    node_id_arg = DeclareLaunchArgument(
        'node_id', default_value='1', description='CANopen Node ID of the device (1-127)')

    eds_file_arg = DeclareLaunchArgument(
        'eds_file',
        default_value=os.path.join(
            get_package_share_directory('canopen_402_driver_ros2'),
            'resource', 'default_cia402_device.eds'
        ),
        description='Path to the EDS file for the CANopen device.'
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='vcan0', description='CAN interface name (e.g., vcan0, can0)')

    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate', default_value='500000', description='CAN interface bitrate in bps')

    sync_interval_arg = DeclareLaunchArgument(
        'sync_interval', default_value='0.01', description='SYNC message interval in seconds (0 to disable)')

    joint_name_arg = DeclareLaunchArgument(
        'joint_name', default_value='motor_joint', description='Name of the joint for JointState messages')

    scale_pos_to_dev_arg = DeclareLaunchArgument('scale_pos_to_dev', default_value='1.0', description='Scaling factor ROS position units to device units.')
    scale_pos_from_dev_arg = DeclareLaunchArgument('scale_pos_from_dev', default_value='1.0', description='Scaling factor device position units to ROS units.')
    scale_vel_to_dev_arg = DeclareLaunchArgument('scale_vel_to_dev', default_value='1.0', description='Scaling factor ROS velocity units to device units.')
    scale_vel_from_dev_arg = DeclareLaunchArgument('scale_vel_from_dev', default_value='1.0', description='Scaling factor device velocity units to ROS units.')
    offset_pos_to_dev_arg = DeclareLaunchArgument('offset_pos_to_dev', default_value='0.0', description='Offset for ROS position units to device units.')
    offset_pos_from_dev_arg = DeclareLaunchArgument('offset_pos_from_dev', default_value='0.0', description='Offset for device position units to ROS units.')
    homing_timeout_seconds_arg = DeclareLaunchArgument('homing_timeout_seconds', default_value='30', description='Timeout for homing procedure in seconds.')

    # --- Node Definition ---
    # The name of the node instance in the ROS graph.
    # The LifecycleNode class name is Cia402LifecycleNode.
    # The executable name is cia402_ros_node (from setup.py entry_points).
    cia402_node_name_config = LaunchConfiguration('cia402_node_name', default='cia402_device')


    cia402_lifecycle_node = LifecycleNode(
        package='canopen_402_driver_ros2',
        executable='cia402_ros_node',
        name=cia402_node_name_config, # Use the LaunchConfiguration for the node name
        namespace='',
        output='screen',
        parameters=[{
            'node_id': LaunchConfiguration('node_id'),
            'eds_file': LaunchConfiguration('eds_file'),
            'can_interface': LaunchConfiguration('can_interface'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'sync_interval': LaunchConfiguration('sync_interval'),
            'joint_name': LaunchConfiguration('joint_name'),
            'scale_pos_to_dev': LaunchConfiguration('scale_pos_to_dev'),
            'scale_pos_from_dev': LaunchConfiguration('scale_pos_from_dev'),
            'scale_vel_to_dev': LaunchConfiguration('scale_vel_to_dev'),
            'scale_vel_from_dev': LaunchConfiguration('scale_vel_from_dev'),
            'offset_pos_to_dev': LaunchConfiguration('offset_pos_to_dev'),
            'offset_pos_from_dev': LaunchConfiguration('offset_pos_from_dev'),
            'homing_timeout_seconds': LaunchConfiguration('homing_timeout_seconds'),
        }]
    )

    # --- Lifecycle Transition Events ---
    # Emit event to request transition to 'configuring' state
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(cia402_lifecycle_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # After 'configuring' completes (node becomes 'inactive'), emit 'activate'
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=cia402_lifecycle_node,
            start_state='configuring', # This is the name of the transition state
            goal_state='inactive',     # This is the primary state reached after TRANSITION_CONFIGURE
            entities=[
                LogInfo(msg="Node configured successfully, requesting activation..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(cia402_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    # Log when node becomes active
    node_active_log_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=cia402_lifecycle_node,
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg="Node is active and operational.")
            ]
        )
    )

    # Optional: Handler to shutdown launch if node reaches 'finalized' state (e.g., after critical error)
    shutdown_on_finalized_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=cia402_lifecycle_node,
            # start_state can be 'errorprocessing' if on_error returns FAILURE, leading to UNCONFIGURED, then cleanup to FINALIZED.
            # Or directly if on_error returns SHUTDOWN. Let's catch 'finalized'.
            goal_state='finalized', # The state reached after TRANSITION_SHUTDOWN from any state, or after cleanup from unconfigured.
            entities=[
                LogInfo(msg="Node reached 'finalized' state, shutting down launch."),
                EmitEvent(event=launch.events.Shutdown())
            ]
        )
    )

    ld = LaunchDescription()
    # Add arguments
    ld.add_action(DeclareLaunchArgument('cia402_node_name', default_value='cia402_device', description="Name for the Cia402LifecycleNode instance"))
    ld.add_action(node_id_arg)
    ld.add_action(eds_file_arg)
    ld.add_action(can_interface_arg)
    ld.add_action(can_bitrate_arg)
    ld.add_action(sync_interval_arg)
    ld.add_action(joint_name_arg)
    ld.add_action(scale_pos_to_dev_arg)
    ld.add_action(scale_pos_from_dev_arg)
    ld.add_action(scale_vel_to_dev_arg)
    ld.add_action(scale_vel_from_dev_arg)
    ld.add_action(offset_pos_to_dev_arg)
    ld.add_action(offset_pos_from_dev_arg)
    ld.add_action(homing_timeout_seconds_arg)

    # Add node
    ld.add_action(cia402_lifecycle_node)

    # Add lifecycle transition handlers
    ld.add_action(configure_event)
    ld.add_action(activate_event_handler)
    ld.add_action(node_active_log_handler)
    ld.add_action(shutdown_on_finalized_handler)

    return ld
