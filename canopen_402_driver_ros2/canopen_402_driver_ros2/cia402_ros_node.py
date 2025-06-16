# canopen_402_driver_ros2/canopen_402_driver_ros2/cia402_ros_node.py
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State as LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import ParameterType, ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor # For main

from std_msgs.msg import String, UInt16, Int8, Float64, Int32
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from canopen_interfaces.srv import COTargetDouble

import canopen
import time
import os

try:
    from canopen_402_driver_py.driver import CiA402Node, PROFILE_POSITION_MODE, PROFILE_VELOCITY_MODE, HOMING_MODE
except ImportError as e:
    print(f"FATAL: Failed to import CiA402Node from canopen_402_driver_py.driver: {e}. Check PYTHONPATH.")
    raise

# Default values for parameters
DEFAULT_NODE_ID = 1; DEFAULT_EDS_FILE = "default_cia402_device.eds"; DEFAULT_CAN_INTERFACE = "vcan0"
DEFAULT_CAN_BITRATE = 500000; DEFAULT_SYNC_INTERVAL = 0.01; DEFAULT_HEARTBEAT_INTERVAL = 0.1
DEFAULT_SCALE_POS_TO_DEV = 1.0; DEFAULT_SCALE_POS_FROM_DEV = 1.0; DEFAULT_SCALE_VEL_TO_DEV = 1.0
DEFAULT_SCALE_VEL_FROM_DEV = 1.0; DEFAULT_OFFSET_POS_TO_DEV = 0.0; DEFAULT_OFFSET_POS_FROM_DEV = 0.0
DEFAULT_HOMING_TIMEOUT_SECONDS = 30; DEFAULT_JOINT_NAME = "motor_joint"

class Cia402LifecycleNode(LifecycleNode):
    def __init__(self, node_name='cia402_lifecycle_node', **kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().info(f"Initializing CiA402 Lifecycle Node '{self.get_name()}' (in Unconfigured state)...")

        # Declare parameters
        self.declare_parameter('node_id', DEFAULT_NODE_ID, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('eds_file', DEFAULT_EDS_FILE, ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('can_interface', DEFAULT_CAN_INTERFACE, ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('can_bitrate', DEFAULT_CAN_BITRATE, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('sync_interval', DEFAULT_SYNC_INTERVAL, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('joint_name', DEFAULT_JOINT_NAME, ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('scale_pos_to_dev', DEFAULT_SCALE_POS_TO_DEV, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('scale_pos_from_dev', DEFAULT_SCALE_POS_FROM_DEV, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('scale_vel_to_dev', DEFAULT_SCALE_VEL_TO_DEV, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('scale_vel_from_dev', DEFAULT_SCALE_VEL_FROM_DEV, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('offset_pos_to_dev', DEFAULT_OFFSET_POS_TO_DEV, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('offset_pos_from_dev', DEFAULT_OFFSET_POS_FROM_DEV, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('homing_timeout_seconds', DEFAULT_HOMING_TIMEOUT_SECONDS, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))

        # Initialize attributes that will be set in on_configure
        self.network = None; self.cia402_node = None; self.status_publish_timer = None
        self.state_publisher = None; self.op_mode_publisher = None; self.statusword_publisher = None
        self.joint_state_publisher = None; self.emcy_publisher = None
        self.nmt_services = {}; self.sm_services = {}; self.om_services = {}
        self.target_subs = {}; self.target_srv = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        try:
            self.node_id = self.get_parameter('node_id').value
            self.eds_file = self.get_parameter('eds_file').value
            # ... (get all other parameters)
            self.can_interface = self.get_parameter('can_interface').value
            self.can_bitrate = self.get_parameter('can_bitrate').value
            self.sync_interval = self.get_parameter('sync_interval').value
            self.joint_name = self.get_parameter('joint_name').value
            self.scale_pos_to_dev = self.get_parameter('scale_pos_to_dev').value
            self.scale_pos_from_dev = self.get_parameter('scale_pos_from_dev').value
            self.scale_vel_to_dev = self.get_parameter('scale_vel_to_dev').value
            self.scale_vel_from_dev = self.get_parameter('scale_vel_from_dev').value
            self.offset_pos_to_dev = self.get_parameter('offset_pos_to_dev').value
            self.offset_pos_from_dev = self.get_parameter('offset_pos_from_dev').value
            self.homing_timeout_s = self.get_parameter('homing_timeout_seconds').value

            self.get_logger().info(f"Parameters: Node ID {self.node_id}, EDS {self.eds_file}, CAN {self.can_interface}@{self.can_bitrate}bps, SYNC {self.sync_interval}s")

            if not os.path.exists(self.eds_file):
                self.get_logger().error(f"EDS file '{self.eds_file}' not found! Configuration failed."); return TransitionCallbackReturn.FAILURE

            self.network = canopen.Network()
            self.network.connect(bustype='socketcan', channel=self.can_interface, bitrate=self.can_bitrate)
            self.get_logger().info(f"Connected to CAN interface {self.can_interface}.")

            self.cia402_node = CiA402Node(self.node_id, self.eds_file, self.network)
            self.get_logger().info(f"CiA402Node driver for ID {self.node_id} initialized.")
            self.cia402_node.initialize_pdos() # Initialize PDOs from device

            # Create publishers
            common_prefix = self.get_name() # Use node name for topics/services
            self.state_publisher = self.create_publisher(String, f'{common_prefix}/cia402_state', 10)
            # ... (create other publishers)
            self.op_mode_publisher = self.create_publisher(String, f'{common_prefix}/current_operation_mode', 10)
            self.statusword_publisher = self.create_publisher(UInt16, f'{common_prefix}/statusword', 10)
            self.joint_state_publisher = self.create_publisher(JointState, f'{common_prefix}/joint_states', 10)
            self.emcy_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10) # Standard topic
            if self.cia402_node: self.cia402_node.add_emcy_callback(self._ros_emcy_callback)

            # Create services and subscribers
            self._init_nmt_services(); self._init_state_machine_services()
            self._init_op_mode_services(); self._init_target_handling()
            self.get_logger().info("ROS interfaces (pubs/subs/srvs) created.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during on_configure: {e}", exc_info=True)
            # Cleanup partially initialized resources if any
            if self.network and self.network.is_connected: self.network.disconnect()
            self.network = None; self.cia402_node = None
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        try:
            if self.cia402_node:
                self.cia402_node.nmt_pre_operational() # Ensure PRE-OP before other actions
                self.get_logger().info(f"Node {self.node_id} NMT state set to PRE-OPERATIONAL.")
                # Consider an "init_motor" sequence here if appropriate for auto-activation
                # For now, just PRE-OP. User can call 'init' service.

            if self.network and self.sync_interval > 0:
                self.network.sync.start(self.sync_interval)
                self.get_logger().info(f"SYNC producer started ({self.sync_interval}s).")

            if not self.status_publish_timer:
                 self.status_publish_timer = self.create_timer(0.1, self.publish_all_status)
            else:
                 self.status_publish_timer.reset() # Ensure it's running
            self.get_logger().info("Status reporting timer activated.")

            super().on_activate(state) # Activate publishers and services
            self.get_logger().info("Node activated successfully.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during on_activate: {e}", exc_info=True)
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        try:
            if self.status_publish_timer and not self.status_publish_timer.canceled:
                self.status_publish_timer.cancel()

            if self.network and self.sync_interval > 0 and self.network.sync.is_running:
                try: self.network.sync.stop()
                except Exception as e: self.get_logger().warn(f"Could not stop SYNC: {e}")

            if self.cia402_node: # Try to bring motor to a safe state
                try:
                    if self.cia402_node.state == 'OPERATION ENABLED': self.cia402_node.quick_stop_command()
                    self.cia402_node.nmt_stop_node()
                except Exception as e: self.get_logger().warn(f"Could not set NMT STOPPED: {e}")

            super().on_deactivate(state) # Deactivates publishers/services
            self.get_logger().info("Node deactivated successfully.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during on_deactivate: {e}", exc_info=True)
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup() is called.")
        try:
            # Destroy timer first
            if self.status_publish_timer:
                self.status_publish_timer.destroy(); self.status_publish_timer = None

            # Destroy publishers, subscribers, services (rclpy handles null checks)
            pub_attrs = ['state_publisher', 'op_mode_publisher', 'statusword_publisher', 'joint_state_publisher', 'emcy_publisher']
            for attr in pub_attrs:
                if hasattr(self, attr) and getattr(self, attr): self.destroy_publisher(getattr(self, attr)); setattr(self, attr, None)

            for srv_dict in [self.nmt_services, self.sm_services, self.om_services]:
                for srv_obj in srv_dict.values(): self.destroy_service(srv_obj)
                srv_dict.clear()
            if self.target_srv: self.destroy_service(self.target_srv); self.target_srv = None
            for sub_obj in self.target_subs.values(): self.destroy_subscription(sub_obj)
            self.target_subs.clear()

            if self.network and self.network.is_connected:
                if self.sync_interval > 0 and self.network.sync.is_running: self.network.sync.stop()
                self.network.disconnect()
            self.network = None; self.cia402_node = None

            self.get_logger().info("Resources cleaned up.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during on_cleanup: {e}", exc_info=True)
            return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        try:
            # Orderly shutdown: deactivate, then cleanup.
            # If current state is active, try to deactivate first.
            if self.get_current_lifecycle_state().label == "active":
                self.on_deactivate(state) # Call our deactivate logic
            # Regardless of current state, try to cleanup.
            self.on_cleanup(state) # Call our cleanup logic

            self.get_logger().info("Node shutdown process complete.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error during on_shutdown: {e}", exc_info=True)
            return TransitionCallbackReturn.ERROR

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f"on_error() is called from state {state.label}.")
        # Attempt cleanup and signal failure, which should lead to UNCONFIGURED or FINALIZED
        try:
            if self.network and self.network.is_connected: self.network.disconnect() # Quick emergency stop for CAN
        except Exception as e_net: self.get_logger().error(f"Emergency CAN disconnect failed in on_error: {e_net}")
        # Don't call full on_cleanup here as it might be too complex or fail again.
        # The goal is to transition out of ERROR state.
        return TransitionCallbackReturn.FAILURE

    def publish_all_status(self):
        if self.cia402_node and self.get_current_lifecycle_state().label == "active":
            try:
                # ... (Publishing logic as before, ensure checks for publisher existence if created in configure)
                if self.state_publisher: self.state_publisher.publish(String(data=str(self.cia402_node.state or "Unknown")))
                # ... (other publishes)
                op_mode_val = self.cia402_node.op_mode
                mode_map = { PROFILE_POSITION_MODE: "Profile Position", PROFILE_VELOCITY_MODE: "Profile Velocity", HOMING_MODE: "Homing" }
                if self.op_mode_publisher: self.op_mode_publisher.publish(String(data=mode_map.get(op_mode_val, f"Unknown/Other ({op_mode_val})") if op_mode_val is not None else "Unavailable"))

                statusword_val = self.cia402_node.get_statusword()
                if self.statusword_publisher and statusword_val is not None: self.statusword_publisher.publish(UInt16(data=statusword_val))

                js_msg = JointState(); js_msg.header.stamp = self.get_clock().now().to_msg(); js_msg.name = [self.joint_name]
                pos_dev = self.cia402_node.get_actual_position(); vel_dev = self.cia402_node.get_actual_velocity(); eff_dev = self.cia402_node.get_actual_torque()
                js_msg.position = [float(pos_dev * self.scale_pos_from_dev + self.offset_pos_from_dev if pos_dev is not None else 0.0)]
                js_msg.velocity = [float(vel_dev * self.scale_vel_from_dev if vel_dev is not None else 0.0)]
                js_msg.effort = [float(eff_dev if eff_dev is not None else 0.0)]
                if self.joint_state_publisher: self.joint_state_publisher.publish(js_msg)

            except Exception as e: self.get_logger().warn(f"Error publishing status: {e}", exc_info=True)

    def _ros_emcy_callback(self, emcy_error: canopen.emcy.EmcyError):
        if not self.emcy_publisher: return # Check if publisher exists (created in on_configure)
        node_id_str = str(self.cia402_node.id) if self.cia402_node else "UnknownID"
        self.get_logger().error(f"EMCY Received from node {node_id_str}: Code {emcy_error.code:#06X}, Reg {emcy_error.register:#04X}, Data {emcy_error.data.hex()}")
        # ... (rest of DiagnosticArray publishing as before) ...
        diag_array = DiagnosticArray(); diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_status = DiagnosticStatus(); diag_status.level = DiagnosticStatus.ERROR
        diag_status.name = f"{self.get_name()}_node_{node_id_str}"
        diag_status.message = f"EMCY Error Code: {emcy_error.code:#06X}"; diag_status.hardware_id = f"CANopenNodeID_{node_id_str}"
        diag_status.values.append(KeyValue(key="EMCY Error Code", value=f"{emcy_error.code:#06X}"))
        diag_status.values.append(KeyValue(key="Error Register", value=f"{emcy_error.register:#04X}"))
        diag_status.values.append(KeyValue(key="Manufacturer Data", value=emcy_error.data.hex()))
        diag_array.status.append(diag_status); self.emcy_publisher.publish(diag_array)


    def _create_trigger_service(self, service_name_suffix, callback_func, storage_dict):
        full_service_name = f"{self.get_name()}/{service_name_suffix}"
        srv = self.create_service(Trigger, full_service_name,
            lambda req, resp: self._handle_trigger_service(req, resp, service_name_suffix, callback_func))
        storage_dict[service_name_suffix] = srv

    def _handle_trigger_service(self, request, response, service_name_suffix, node_method_callable):
        if not self.cia402_node:
            response.success = False; response.message = "CiA402Node driver not initialized."
            self.get_logger().error(f"Service '{service_name_suffix}' failed: Driver not ready."); return response
        if self.get_current_lifecycle_state().label != "active":
            response.success = False; response.message = f"Node is not in 'active' state (current: {self.get_current_lifecycle_state().label})."
            self.get_logger().warn(f"Service '{service_name_suffix}' rejected: Node not active."); return response
        try:
            self.get_logger().info(f"Service '{service_name_suffix}' called.")
            result = node_method_callable()
            response.success = result if isinstance(result, bool) else True
            response.message = f"Service '{service_name_suffix}' {'succeeded' if response.success else 'failed by driver'}."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False; response.message = f"Exception in service '{service_name_suffix}': {e}"
            self.get_logger().error(response.message, exc_info=True)
        return response

    # ... (_init_nmt_services, _init_state_machine_services, _handle_init_motor, _init_op_mode_services - adjust to use storage_dict)
    def _init_nmt_services(self):
        self._create_trigger_service("nmt_start_node", lambda: self.cia402_node.nmt_start_node(), self.nmt_services)
        self._create_trigger_service("nmt_stop_node", lambda: self.cia402_node.nmt_stop_node(), self.nmt_services)
        self._create_trigger_service("nmt_pre_operational", lambda: self.cia402_node.nmt_pre_operational(), self.nmt_services)
        self._create_trigger_service("nmt_reset_node", lambda: self.cia402_node.nmt_reset_node(), self.nmt_services)
        self._create_trigger_service("nmt_reset_communication", lambda: self.cia402_node.nmt_reset_communication(), self.nmt_services)
        self.get_logger().debug("NMT services created.")

    def _init_state_machine_services(self):
        self._create_trigger_service("init", self._handle_init_motor, self.sm_services) # Renamed to "init"
        self._create_trigger_service("enable_operation", lambda: self.cia402_node.enable_operation(), self.sm_services)
        self._create_trigger_service("disable_operation", lambda: self.cia402_node.disable_operation(), self.sm_services)
        self._create_trigger_service("shutdown", lambda: self.cia402_node.shutdown(), self.sm_services)
        self._create_trigger_service("quick_stop", lambda: self.cia402_node.quick_stop_command(), self.sm_services)
        self._create_trigger_service("fault_reset", lambda: self.cia402_node.fault_reset(), self.sm_services)
        self._create_trigger_service("recover", lambda: self.cia402_node.fault_reset(), self.sm_services)
        self._create_trigger_service("halt", lambda: self.cia402_node.quick_stop_command(), self.sm_services)
        self.get_logger().debug("State machine services created.")

    def _handle_init_motor(self):
        # ... (as before) ...
        if not self.cia402_node: return False
        self.get_logger().info("Executing init_motor sequence...")
        try:
            self.cia402_node.nmt_pre_operational(); time.sleep(0.05)
            if self.cia402_node.state == 'FAULT': self.cia402_node.fault_reset(); time.sleep(0.1)
            if self.cia402_node.state == 'FAULT': self.get_logger().error("Failed to reset fault."); return False
            self.cia402_node.shutdown(); time.sleep(0.05)
            self.cia402_node.switch_on(); time.sleep(0.05)
            self.get_logger().info(f"init_motor sequence completed. Current state: {self.cia402_node.state}")
            return True
        except Exception as e: self.get_logger().error(f"Exception in _handle_init_motor: {e}", exc_info=True); return False


    def _init_op_mode_services(self):
        self._create_trigger_service("profile_position_mode", lambda: self.cia402_node.set_op_mode(PROFILE_POSITION_MODE), self.om_services)
        self._create_trigger_service("profile_velocity_mode", lambda: self.cia402_node.set_op_mode(PROFILE_VELOCITY_MODE), self.om_services)
        self._create_trigger_service("homing_mode", lambda: self.cia402_node.set_op_mode(HOMING_MODE), self.om_services)
        self.get_logger().debug("Operation mode set services created.")

    def _init_target_handling(self):
        common_prefix = self.get_name()
        self.target_subs['pos'] = self.create_subscription(Float64, f'{common_prefix}/target_position', self._target_position_callback, 10)
        self.target_subs['vel'] = self.create_subscription(Float64, f'{common_prefix}/target_velocity', self._target_velocity_callback, 10)
        self.target_srv = self.create_service(COTargetDouble, f"{common_prefix}/target", self._handle_set_target_service)
        self.get_logger().debug("Target handling (subscribers and service) created.")

    # Target Callbacks & Service Handler - check for active lifecycle state
    def _target_position_callback(self, msg: Float64):
        if self.get_current_lifecycle_state().label != "active": self.get_logger().debug("Target pos ignored: Node not active."); return
        # ... (rest of method as before)
        if not self.cia402_node or self.cia402_node.op_mode != PROFILE_POSITION_MODE:
            self.get_logger().warn(f"Target pos {msg.data} ignored: Not in PP Mode (is {self.cia402_node.op_mode if self.cia402_node else 'N/A'})."); return
        if self.cia402_node.state != 'OPERATION ENABLED':
            self.get_logger().warn(f"Target pos {msg.data} ignored: Not in OP_ENABLED (is {self.cia402_node.state})."); return
        try:
            device_target_pos = msg.data * self.scale_pos_to_dev + self.offset_pos_to_dev
            self.get_logger().info(f"ROS Target Pos: {msg.data} -> Device Target Pos: {int(device_target_pos)}. Setting & triggering.")
            self.cia402_node.set_profile_position_target_location(int(device_target_pos))
            self.cia402_node.trigger_profile_position_movement()
        except Exception as e: self.get_logger().error(f"Error processing target_position: {e}", exc_info=True)


    def _target_velocity_callback(self, msg: Float64):
        if self.get_current_lifecycle_state().label != "active": self.get_logger().debug("Target vel ignored: Node not active."); return
        # ... (rest of method as before)
        if not self.cia402_node or self.cia402_node.op_mode != PROFILE_VELOCITY_MODE:
            self.get_logger().warn(f"Target vel {msg.data} ignored: Not in PV Mode (is {self.cia402_node.op_mode if self.cia402_node else 'N/A'})."); return
        if self.cia402_node.state != 'OPERATION ENABLED':
             self.get_logger().warn(f"Target vel {msg.data} ignored: Not in OP_ENABLED (is {self.cia402_node.state})."); return
        try:
            device_target_vel = msg.data * self.scale_vel_to_dev
            self.get_logger().info(f"ROS Target Vel: {msg.data} -> Device Target Vel: {int(device_target_vel)}. Setting.")
            self.cia402_node.set_profile_velocity_target_velocity(int(device_target_vel))
        except Exception as e: self.get_logger().error(f"Error processing target_velocity: {e}", exc_info=True)


    def _handle_set_target_service(self, request: COTargetDouble.Request, response: COTargetDouble.Response):
        if self.get_current_lifecycle_state().label != "active":
             self.get_logger().warn("Set_target service call failed: Node not active.")
             response.success = False; return response
        # ... (rest of method as before)
        if not self.cia402_node: response.success = False; return response
        current_op_mode = self.cia402_node.op_mode; target_value_ros = request.target
        device_target_value = 0.0; processed = False
        try:
            if current_op_mode == PROFILE_POSITION_MODE:
                device_target_value = target_value_ros * self.scale_pos_to_dev + self.offset_pos_to_dev
                self.cia402_node.set_profile_position_target_location(int(device_target_value))
                self.cia402_node.trigger_profile_position_movement(); processed = True
            elif current_op_mode == PROFILE_VELOCITY_MODE:
                device_target_value = target_value_ros * self.scale_vel_to_dev
                self.cia402_node.set_profile_velocity_target_velocity(int(device_target_value)); processed = True
            else: self.get_logger().warn(f"Set_target: Mode {current_op_mode} not handled."); response.success = False; return response
            if processed: response.success = True; self.get_logger().info(f"Set_target: Mode={current_op_mode}, ROS Target={target_value_ros}, Device Target={int(device_target_value)} -> Success.")
        except Exception as e: self.get_logger().error(f"Error in set_target for mode {current_op_mode}: {e}", exc_info=True); response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = Cia402LifecycleNode()
    executor.add_node(node)
    try:
        # executor.spin() will not automatically transition states.
        # Use ros2 lifecycle command line tools to manage the node.
        # e.g. ros2 lifecycle set <node_name> configure
        #      ros2 lifecycle set <node_name> activate
        # For testing, one could programmatically trigger transitions here after spinning executor in a thread.
        # For now, just spin and let external tools manage.
        node.get_logger().info("Cia402LifecycleNode created. Use 'ros2 lifecycle' commands to manage.")
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Executor shutting down or keyboard interrupt.")
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception in main: {e}", exc_info=True)
        else: print(f"FATAL: Unhandled exception in main before node init: {e}")
    finally:
        if node and not node.is_destroyed: # Check if node exists and not already destroyed
            # Attempt graceful shutdown through lifecycle transitions if node is still managed by executor
            # This is complex if spin() was interrupted abruptly.
            # Simplest is to ensure destroy_node is called.
            if node.get_current_lifecycle_state().label != "finalized":
                 node.get_logger().info("Manually ensuring node resources are released via destroy_node.")
                 node.destroy_node() # This will call on_shutdown if not already called by executor

        # Important: Shutdown executor after its nodes are destroyed or no longer needed.
        # if executor: executor.shutdown() # Shutdown executor if it's still alive
        # rclpy.shutdown() should handle executor shutdown if it was registered.
        # For a single node like this, node.destroy_node() then rclpy.shutdown() is typical.

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
