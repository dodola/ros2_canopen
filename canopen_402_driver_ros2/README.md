# ROS 2 CiA 402 CANopen Driver Node

This package provides a ROS 2 Python lifecycle node for controlling CiA 402 compliant CANopen devices. It acts as a bridge between the ROS 2 ecosystem and a CANopen motor drive, utilizing the `python-canopen` library and the custom `canopen_402_driver_py` Python driver.

## Overview

The `cia402_lifecycle_node` (executable: `cia402_ros_node`) allows users to:
- Initialize and manage the lifecycle of a CANopen CiA 402 device.
- Control the NMT state and CiA 402 state machine of the device.
- Set various operation modes (Profiled Position, Profiled Velocity, Homing, etc.).
- Send target commands (position, velocity) via ROS topics or services.
- Receive status information (actual position, velocity, torque, device state, statusword) via ROS topics.
- Handle and report EMCY (Emergency) messages from the device.
- Configure device parameters, scaling, and PDO mappings (future enhancement).

## Dependencies

- **ROS 2 Humble Hawksbill** (or newer, adjust as needed)
- **`rclpy`**: ROS 2 Python client library.
- **`canopen`**: The `python-canopen` library. Install via pip: `pip install canopen`.
- **`python-can`**: Backend for `python-canopen`. Install via pip: `pip install python-can` (and any necessary system drivers for your CAN hardware, e.g., `sudo apt install can-utils` for SocketCAN).
- **`canopen_402_driver_py`**: The underlying Python driver for CiA 402 logic. This package should be in the same ROS 2 workspace.
- **`sensor_msgs`**: For `JointState` messages.
- **`std_msgs`**: For basic status messages.
- **`std_srvs`**: For `Trigger` services.
- **`diagnostic_msgs`**: For publishing EMCY messages.
- **`canopen_interfaces`**: For services like `COTargetDouble` (if used, currently defined).

## Build Instructions

1.  Ensure all dependencies are installed.
2.  Place this package (`canopen_402_driver_ros2`) and the `canopen_402_driver_py` package into the `src` directory of your ROS 2 workspace.
3.  Build the workspace:
    ```bash
    cd /path/to/your_ros2_workspace
    colcon build --symlink-install
    ```
4.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Configuration

The node is launched using `cia402_node.launch.py`. Key parameters can be set via launch arguments:

-   **`node_id`**: (int, default: 1) The CANopen Node ID of your device.
-   **`eds_file`**: (string, default: `default_cia402_device.eds` within the package) **Crucial:** Path to the EDS (Electronic Data Sheet) file for your specific CANopen device. The default is a minimal dummy file. **You MUST provide a valid EDS for your hardware.**
-   **`can_interface`**: (string, default: `vcan0`) The name of the CAN interface to use (e.g., `can0`, `socketcan:can0`, `pcan:PCAN_USBBUS1`).
-   **`can_bitrate`**: (int, default: 500000) The bitrate of the CAN bus in bps.
-   **`sync_interval`**: (double, default: 0.01) The interval in seconds for sending SYNC messages. Set to 0 to disable SYNC production by this node.
-   **`joint_name`**: (string, default: `motor_joint`) The name used for the joint in `sensor_msgs/JointState` messages.
-   **Scaling/Offset Parameters**:
    -   `scale_pos_to_dev`, `scale_pos_from_dev`
    -   `scale_vel_to_dev`, `scale_vel_from_dev`
    -   `offset_pos_to_dev`, `offset_pos_from_dev`
    (Defaults are 1.0 or 0.0. These **must be configured** based on your device's units and desired ROS units, e.g., radians for position, rad/s for velocity).
-   **`homing_timeout_seconds`**: (int, default: 30) Timeout for homing operations (used by the underlying driver).

**Example EDS File:**
A very basic `default_cia402_device.eds` is provided in the `resource` directory. This is primarily for structural testing and **should be replaced by your device's actual EDS file.**

## Usage

### Launching the Node

```bash
ros2 launch canopen_402_driver_ros2 cia402_node.launch.py \
    node_id:=5 \
    eds_file:="/path/to/your/device.eds" \
    can_interface:="can0" \
    joint_name:="axis_1" \
    scale_pos_from_dev:=0.0001 # Example: 10000 counts/revolution -> 2pi/10000 rad/count
```

The launch file attempts to automatically transition the node to the `active` state.

### Lifecycle Management

The node is a lifecycle node. You can manage its state using the `ros2 lifecycle` command-line tool:

-   **Check current state:**
    ```bash
    ros2 lifecycle get /cia402_device
    ```
    (Replace `/cia402_device` with the actual node name if changed in the launch file, e.g. using `cia402_node_name:=my_motor_node` launch argument).
-   **Manually trigger transitions (if auto-transitions in launch file are disabled or if needed for recovery):**
    ```bash
    ros2 lifecycle set /cia402_device configure
    ros2 lifecycle set /cia402_device activate
    ros2 lifecycle set /cia402_device deactivate
    ros2 lifecycle set /cia402_device cleanup
    ros2 lifecycle set /cia402_device shutdown
    ```

### Interacting with the Node

**Services:**
Service names are typically `/{node_name}/{service_suffix}`. Example for node name `cia402_device`:

-   **Initialize Motor (sequence of state changes):**
    ```bash
    ros2 service call /cia402_device/init std_srvs/srv/Trigger "{}"
    ```
-   **Enable Operation:**
    ```bash
    ros2 service call /cia402_device/enable_operation std_srvs/srv/Trigger "{}"
    ```
-   **Set Profile Position Mode:**
    ```bash
    ros2 service call /cia402_device/profile_position_mode std_srvs/srv/Trigger "{}"
    ```
-   **Set Target (Generic, uses `COTargetDouble.srv`):**
    (Assumes node is in an appropriate mode like Profile Position or Profile Velocity and state is Operation Enabled)
    ```bash
    ros2 service call /cia402_device/target canopen_interfaces/srv/COTargetDouble "{target: 1.57}" # e.g., 1.57 radians or rad/s
    ```
-   **List all services for the node:**
    ```bash
    ros2 service list | grep /cia402_device
    ```

**Topics:**
Topic names are typically `/{node_name}/topic_name`.

-   **Target Position (Profiled Position Mode):**
    ```bash
    ros2 topic pub --once /cia402_device/target_position std_msgs/msg/Float64 "{data: 3.14}"
    ```
-   **Target Velocity (Profiled Velocity Mode):**
    ```bash
    ros2 topic pub --once /cia402_device/target_velocity std_msgs/msg/Float64 "{data: 0.5}"
    ```
-   **Listen to Joint States:**
    ```bash
    ros2 topic echo /cia402_device/joint_states
    ```
-   **Listen to CiA 402 State:**
    ```bash
    ros2 topic echo /cia402_device/cia402_state
    ```
-   **Listen to Statusword:**
    ```bash
    ros2 topic echo /cia402_device/statusword
    ```
-   **Listen to Diagnostics (for EMCY messages):**
    ```bash
    ros2 topic echo /diagnostics
    ```

## Troubleshooting

-   **Node Fails to Configure/Activate:**
    -   Check `eds_file` path and validity. The EDS file is critical.
    -   Verify `can_interface` is correct and the CAN bus is operational (e.g., using `can-utils` like `cansend`, `candump`).
    -   Ensure `node_id` matches the device's configuration.
    -   Check ROS 2 logs for detailed error messages.
-   **No Communication / SYNC Timeout:**
    -   Verify CAN bitrate matches all devices on the bus.
    -   Ensure SYNC producer is active if devices require it (controlled by `sync_interval`).
-   **Incorrect Scaling:**
    -   Carefully determine the correct scaling and offset parameters for your device to convert between device-specific units (e.g., encoder counts, counts/s) and ROS standard units (radians, rad/s).
