import canopen
from canopen.profiles import cia_402
from canopen.objectdictionary import ODVariable, ODArray, ODRecord
import time

# Define constants for common CiA 402 operation modes
PROFILE_POSITION_MODE = 1
PROFILE_VELOCITY_MODE = 3
PROFILE_TORQUE_MODE = 4
HOMING_MODE = 6
INTERPOLATED_POSITION_MODE = 7
CYCLIC_SYNCHRONOUS_POSITION_MODE = 8
CYCLIC_SYNCHRONOUS_VELOCITY_MODE = 9
CYCLIC_SYNCHRONOUS_TORQUE_MODE = 10

class CiA402Node(canopen.RemoteNode):
    def __init__(self, node_id, eds_file, network):
        super().__init__(node_id, eds_file)
        self.cia402 = cia_402.Node402(self)
        network.add_node(self)
        # PDOs should be initialized after network connection and node is ready.
        # self.initialize_pdos() # Call this externally after network.connect()
        self.emcy_callbacks = [] # For custom EMCY handling

    def get_statusword(self):
        try:
            return self.sdo[0x6041].raw
        except Exception as e:
            print(f"Error reading Statusword (0x6041): {e}"); return None

    @property
    def state(self):
        return self.cia402.state # Relies on profile to read SW and determine state

    def send_controlword(self, control_value): # Direct SDO write
        try:
            self.sdo[0x6040].raw = int(control_value)
        except Exception as e:
            print(f"Error writing Controlword (0x6040): {e}")

    def nmt_start_node(self): self.nmt.state = 'OPERATIONAL'
    def nmt_stop_node(self): self.nmt.state = 'STOPPED'
    def nmt_pre_operational(self): self.nmt.state = 'PRE-OPERATIONAL'
    def nmt_reset_node(self): self.nmt.state = 'RESET'
    def nmt_reset_communication(self): self.nmt.state = 'RESET COMMUNICATION'

    def transition_to_state(self, target_state):
        try:
            self.cia402.state = target_state
        except Exception as e:
            print(f"Error transitioning to state {target_state}: {e}")

    def shutdown(self): self.transition_to_state('READY TO SWITCH ON')
    def switch_on(self): self.transition_to_state('SWITCHED ON')
    def enable_operation(self): self.transition_to_state('OPERATION ENABLED')
    def disable_operation(self): self.transition_to_state('SWITCHED ON')
    def quick_stop_command(self): self.transition_to_state('QUICK STOP ACTIVE')

    def fault_reset(self):
        if self.state == 'FAULT':
            try:
                self.cia402.reset_fault()
                print("Fault reset command sent.")
            except Exception as e:
                print(f"Error sending fault reset: {e}")
        else:
            print(f"Node is not in FAULT state (current: {self.state}). No fault reset sent.")

    @property
    def op_mode(self):
        try:
            return self.cia402.op_mode
        except Exception as e:
            print(f"Error reading op_mode via profile: {e}"); return None

    def set_op_mode(self, mode_value):
        try:
            self.cia402.op_mode = mode_value
            print(f"Requested operation mode change to {mode_value}.")
        except Exception as e:
            print(f"Error setting op_mode to {mode_value} via profile: {e}")

    @property
    def supported_op_modes(self):
        modes = []
        try:
            supported_modes_obj = self.sdo[0x6502]
            raw_value = 0
            if isinstance(supported_modes_obj, ODVariable):
                raw_value = supported_modes_obj.raw
            elif isinstance(supported_modes_obj, (ODArray, ODRecord)) and 1 in supported_modes_obj:
                 raw_value = supported_modes_obj[1].raw

            if raw_value != 0:
                for i in range(32):
                    if (raw_value >> i) & 1:
                        modes.append(i + 1)
            elif isinstance(supported_modes_obj, (ODArray, ODRecord)):
                 for sub_idx_key_or_obj in supported_modes_obj:
                    try:
                        val_to_check = None
                        current_entry = supported_modes_obj[sub_idx_key_or_obj] if isinstance(sub_idx_key_or_obj, int) else sub_idx_key_or_obj
                        if hasattr(current_entry, 'raw'):
                             val_to_check = current_entry.raw
                        if val_to_check is not None and val_to_check != 0:
                             modes.append(val_to_check)
                    except (KeyError, canopen.SdoAbortedError, AttributeError, TypeError):
                        continue
        except KeyError: print("Warning: Object 0x6502 (Supported drive modes) not found in OD.")
        except canopen.SdoAbortedError as e: print(f"SDO Aborted Error reading 0x6502: {e}")
        except Exception as e: print(f"Unexpected error reading supported_op_modes (0x6502): {e.__class__.__name__} - {e}")
        return list(set(modes))

    # --- Profiled Position Mode (PP - Mode 1) ---
    def set_profile_position_target_location(self, target_pos: int): self.sdo[0x607A].raw = int(target_pos)
    def get_profile_position_target_location(self): return self.sdo[0x607A].raw
    def set_profile_position_velocity(self, velocity: int): self.sdo[0x6081].raw = int(velocity)
    def get_profile_position_velocity(self): return self.sdo[0x6081].raw
    def set_profile_acceleration(self, accel: int): self.sdo[0x6083].raw = int(accel)
    def get_profile_acceleration(self): return self.sdo[0x6083].raw
    def set_profile_deceleration(self, decel: int): self.sdo[0x6084].raw = int(decel)
    def get_profile_deceleration(self): return self.sdo[0x6084].raw

    def trigger_profile_position_movement(self, change_set_immediately=True):
        if self.state != 'OPERATION ENABLED' or self.op_mode != PROFILE_POSITION_MODE:
            print(f"Error: Device must be in 'OPERATION ENABLED' ({self.state}) and 'PROFILE POSITION MODE' ({self.op_mode}) to trigger movement.")
            return
        try:
            self.cia402.command(command='ENABLE_OPERATION', new_set_point=False, change_set_immediately=change_set_immediately)
            self.cia402.command(command='ENABLE_OPERATION', new_set_point=True, change_set_immediately=change_set_immediately)
            self.cia402.command(command='ENABLE_OPERATION', new_set_point=False, change_set_immediately=change_set_immediately)
            print(f"PP movement triggered.")
        except Exception as e: print(f"Error triggering PP movement: {e}")

    def is_target_reached_pp(self): status = self.get_statusword(); return bool((status >> 10) & 1) if status is not None else False

    # --- Profiled Velocity Mode (PV - Mode 3) ---
    def set_profile_velocity_target_velocity(self, target_vel: int): self.sdo[0x60FF].raw = int(target_vel)
    def get_profile_velocity_target_velocity(self): return self.sdo[0x60FF].raw
    def set_max_profile_velocity_pv(self, max_vel: int): self.set_profile_position_velocity(max_vel)
    def get_max_profile_velocity_pv(self): return self.get_profile_position_velocity()

    def trigger_profile_velocity_movement(self):
        if self.state != 'OPERATION ENABLED':
            print(f"Warning: Device not in 'OPERATION ENABLED' (current: {self.state}). Attempting to enable.")
            self.enable_operation()
        if self.op_mode != PROFILE_VELOCITY_MODE:
            print(f"Error: Device not in 'PROFILE VELOCITY MODE' (current: {self.op_mode}). Cannot ensure PV movement conditions.")
            return
        print("Device is in/transitioned to OPERATION ENABLED and is in PV mode. Movement depends on Target Velocity (0x60FF) value.")

    def stop_profile_velocity_movement(self):
        if self.op_mode == PROFILE_VELOCITY_MODE:
            if self.state == 'OPERATION ENABLED': self.set_profile_velocity_target_velocity(0); print("PV stop requested.")
            else: print(f"Warning: State is {self.state}, not OPERATION ENABLED for PV stop.")
        else: print("Warning: Not in PV Mode for stop.")

    def is_target_velocity_reached_pv(self): status = self.get_statusword(); return bool((status >> 10) & 1) if status is not None else False
    def is_standstill_pv(self): status = self.get_statusword(); return bool((status >> 12) & 1) if status is not None else False

    # --- Homing Mode (HM - Mode 6) ---
    def set_homing_method(self, method: int): self.sdo[0x6098].raw = int(method)
    def get_homing_method(self): return self.sdo[0x6098].raw
    def set_homing_speeds(self, speed_sw: int, speed_zero: int): self.sdo[0x6099][1].raw = int(speed_sw); self.sdo[0x6099][2].raw = int(speed_zero)
    def get_homing_speeds(self): return (self.sdo[0x6099][1].raw, self.sdo[0x6099][2].raw)
    def set_homing_acceleration(self, accel: int): self.sdo[0x609A].raw = int(accel)
    def get_homing_acceleration(self): return self.sdo[0x609A].raw

    def start_homing_procedure(self):
        if self.state != 'OPERATION ENABLED' or self.op_mode != HOMING_MODE:
            print(f"Error: Device must be in 'OPERATION ENABLED' ({self.state}) and 'HOMING MODE' ({self.op_mode}) to start homing.")
            return
        try:
            base_op_enabled_cw = 0x000F
            current_cw = self.sdo[0x6040].raw
            cw_homing_step_0 = (current_cw & 0xFFF0) | base_op_enabled_cw & ~(1 << 4)
            self.send_controlword(cw_homing_step_0)
            # time.sleep(0.01) # Generally not needed for SDOs with python-canopen
            cw_homing_step_1 = cw_homing_step_0 | (1 << 4)
            self.send_controlword(cw_homing_step_1)
            print("Homing procedure started (CW bit 4 set to 1 after ensuring it was 0).")
        except Exception as e: print(f"Error starting homing: {e}")

    def is_homing_attained(self): status = self.get_statusword(); return bool((status >> 12) & 1) if status is not None else False
    def is_homing_error(self): status = self.get_statusword(); return bool((status >> 13) & 1) if status is not None else False

    # --- SDO Data Acquisition ---
    def get_actual_position(self):
        try: return self.sdo[0x6064].raw
        except Exception as e: print(f"Error SDO read 0x6064: {e}"); return None
    def get_actual_velocity(self):
        try: return self.sdo[0x606C].raw
        except Exception as e: print(f"Error SDO read 0x606C: {e}"); return None
    def get_actual_torque(self):
        try: return self.sdo[0x6077].raw
        except Exception as e: print(f"Error SDO read 0x6077: {e}"); return None
    def get_digital_inputs(self):
        try: return self.sdo[0x60FD].raw
        except Exception as e: print(f"Error SDO read 0x60FD: {e}"); return None

    # --- PDO Configuration and Usage ---
    def initialize_pdos(self):
        print("Initializing PDOs by reading current configuration from device...")
        if self.nmt.state not in ['PRE-OPERATIONAL', 'OPERATIONAL']:
            print(f"Warning: Node NMT state is {self.nmt.state}. SDO comm for PDO config may fail. Best in PRE-OPERATIONAL.")
        try:
            self.tpdo.read()
            self.rpdo.read()
            # print("TPDOs current configuration:"); [self.tpdo[i].log_configuration() for i in self.tpdo]
            # print("RPDOs current configuration:"); [self.rpdo[i].log_configuration() for i in self.rpdo]
        except Exception as e: print(f"Error reading PDO configuration from device: {e}")

    def configure_tpdo(self, pdo_idx: int, mappings: list, trans_type=254, event_timer=0, enabled=True):
        if pdo_idx not in self.tpdo: print(f"Error: TPDO{pdo_idx} not found."); return False
        tpdo = self.tpdo[pdo_idx]
        try:
            tpdo.stop(); tpdo.clear()
            for item in mappings:
                if isinstance(item, str): tpdo.add_variable(item)
                elif isinstance(item, tuple) and len(item)==2: tpdo.add_variable(item[0], item[1])
            tpdo.trans_type = trans_type
            if trans_type >= 254 or trans_type == 0: tpdo.event_timer = event_timer
            tpdo.enabled = enabled; tpdo.save()
            # tpdo.log_configuration()
            if enabled: tpdo.start()
            print(f"TPDO{pdo_idx} configured. Enabled: {enabled}"); return True
        except Exception as e: print(f"Error config TPDO{pdo_idx}: {e}"); return False

    def configure_rpdo(self, pdo_idx: int, mappings: list, enabled=True, trans_type=None):
        if pdo_idx not in self.rpdo: print(f"Error: RPDO{pdo_idx} not found."); return False
        rpdo = self.rpdo[pdo_idx]
        try:
            rpdo.stop(); rpdo.clear()
            for item in mappings:
                if isinstance(item, str): rpdo.add_variable(item)
                elif isinstance(item, tuple) and len(item)==2: rpdo.add_variable(item[0], item[1])
            if trans_type is not None: rpdo.trans_type = trans_type
            rpdo.enabled = enabled; rpdo.save()
            # rpdo.log_configuration()
            if enabled: rpdo.start()
            print(f"RPDO{pdo_idx} configured. Enabled: {enabled}"); return True
        except Exception as e: print(f"Error config RPDO{pdo_idx}: {e}"); return False

    def _find_pdo_variable_key(self, pdo_map_obj, var_identifier):
        if isinstance(var_identifier, str): return var_identifier
        if isinstance(var_identifier, tuple) and len(var_identifier) == 2:
            idx, subidx = var_identifier
            for var_info in pdo_map_obj.map:
                if var_info.index == idx and var_info.subindex == subidx: return var_info.name
        print(f"Could not find/map variable identifier: {var_identifier}"); return None

    def get_tpdo_variable_value(self, pdo_idx: int, var_identifier, timeout=0.1): # timeout default
        if pdo_idx not in self.tpdo: print(f"TPDO{pdo_idx} not found."); return None
        tpdo = self.tpdo[pdo_idx]; key = self._find_pdo_variable_key(tpdo, var_identifier)
        if not key: print(f"Var {var_identifier} not in TPDO{pdo_idx}."); return None
        try:
            tpdo.wait_for_reception(timeout)
            return tpdo[key].raw
        except canopen.pdo.PdoError as e: print(f"PDO Timeout/Error TPDO{pdo_idx} var {key}: {e}"); return None
        except Exception as e: print(f"Error get TPDO{pdo_idx} var {key}: {e}"); return None

    def set_rpdo_variable_value(self, pdo_idx: int, var_identifier, value):
        if pdo_idx not in self.rpdo: print(f"RPDO{pdo_idx} not found."); return False
        rpdo = self.rpdo[pdo_idx]; key = self._find_pdo_variable_key(rpdo, var_identifier)
        if not key: print(f"Var {var_identifier} not in RPDO{pdo_idx}."); return False
        try:
            rpdo[key].raw = value
            # Transmission logic: if master sends RPDO to slave, transmit() is appropriate.
            # If this RPDO is for slave to send to master (i.e., a misnomer, should be TPDO on slave),
            # then transmit() is not used here by master.
            # Assuming this RPDO is for master->slave communication:
            rpdo.transmit()
            return True
        except Exception as e: print(f"Error set RPDO{pdo_idx} var {key}: {e}"); return False

    # --- Error Handling and Diagnostics ---
    def get_error_register(self):
        """Reads the Error Register (0x1001) via SDO. Returns integer or None."""
        try:
            return self.sdo[0x1001].raw
        except KeyError: print("Error: OD[0x1001] (Error Register) not found."); return None
        except Exception as e: print(f"Error SDO read 0x1001: {e}"); return None

    def get_predefined_error_field_count(self):
        """Reads count of errors in Pre-defined Error Field (0x1003,0) via SDO. Returns int or None."""
        try:
            return self.sdo[0x1003][0].raw
        except KeyError: print("Error: OD[0x1003][0] (ErrField count) not found."); return None
        except Exception as e: print(f"Error SDO read 0x1003[0]: {e}"); return None

    def get_predefined_error(self, error_idx: int):
        """Reads error code from Pre-defined Error Field (0x1003,error_idx). 1-based error_idx."""
        if error_idx < 1: print("Error: error_idx must be 1-based."); return None
        try:
            num_errors = self.get_predefined_error_field_count()
            if num_errors is None or error_idx > num_errors:
                # print(f"Error: error_idx {error_idx} OOB (max: {num_errors or 0}).") # Can be noisy
                return None # No error at this index or count is zero/None
            return self.sdo[0x1003][error_idx].raw
        except KeyError: print(f"Error: OD[0x1003][{error_idx}] not found."); return None
        except Exception as e: print(f"Error SDO read 0x1003[{error_idx}]: {e}"); return None

    _default_emcy_dispatcher_registered = False
    def _internal_emcy_dispatcher(self, emcy_error):
        """Internal dispatcher to call all custom callbacks."""
        # Default printing action
        print(f"EMCY Received (Node {self.id}): Code {emcy_error.code:#06X}, Reg {emcy_error.register:#04X}, Data {emcy_error.data.hex()}, TS {emcy_error.timestamp}")
        for cb in self.emcy_callbacks:
            try: cb(emcy_error)
            except Exception as e: print(f"Error in custom EMCY callback {cb.__name__}: {e}")

    def add_emcy_callback(self, callback):
        """Adds a custom callback for EMCY messages from this node."""
        if not callable(callback): print("Error: EMCY callback not callable."); return
        if not CiA402Node._default_emcy_dispatcher_registered:
            # Register the internal dispatcher ONCE for this node's EMCY service.
            # This ensures the default print action and subsequent custom callbacks are handled.
            try:
                self.emcy.add_callback(self._internal_emcy_dispatcher)
                CiA402Node._default_emcy_dispatcher_registered = True # Class level flag for simplicity
                print(f"Registered internal EMCY dispatcher for node {self.id}.")
            except Exception as e:
                print(f"Error registering internal EMCY dispatcher: {e}")
                return # Cannot proceed if dispatcher fails

        if callback not in self.emcy_callbacks:
            self.emcy_callbacks.append(callback)
            print(f"Custom EMCY callback {callback.__name__} added for node {self.id}.")
        else:
            print(f"Custom EMCY callback {callback.__name__} already registered for node {self.id}.")

    def remove_emcy_callback(self, callback):
        """Removes a custom EMCY callback."""
        try:
            self.emcy_callbacks.remove(callback)
            print(f"Custom EMCY callback {callback.__name__} removed for node {self.id}.")
        except ValueError:
            print(f"Custom EMCY callback {callback.__name__} not found for node {self.id}.")


if __name__ == '__main__':
    print("CiA402Node class with Error/Diagnostics methods defined.")
    # Example (pseudo-code, requires full setup):
    # network = canopen.Network()
    # node = None
    # try:
    #     network.connect(bustype='socketcan', channel='vcan0', bitrate=250000)
    #     node = CiA402Node(1, 'your_device.eds', network)
    #     node.nmt_pre_operational()
    #     node.wait_for_heartbeat(timeout=2)
    #
    #     def my_emcy_handler(emcy_error): # User's custom handler
    #         print(f"**Custom Handler Got EMCY**: Code {emcy_error.code:#06X}")
    #
    #     node.add_emcy_callback(my_emcy_handler) # Add custom handler
    #     # The internal dispatcher (with default print) gets registered on first add_emcy_callback.
    #
    #     err_reg = node.get_error_register()
    #     if err_reg is not None: print(f"Error Register (0x1001): {err_reg:#04X} (Bit0:Generic, Bit1:Current, Bit2:Voltage, Bit3:Temp, Bit4:Comm, Bit5:DevProfile, Bit7:Manufacturer)")
    #
    #     err_count = node.get_predefined_error_field_count()
    #     if err_count is not None:
    #         print(f"Pre-defined Error Field (0x1003) has {err_count} error(s):")
    #         for i in range(1, err_count + 1):
    #             err_code = node.get_predefined_error(i)
    #             if err_code is not None: print(f"  Error[{i}]: {err_code:#010X}")
    #
    #     # print("Intentionally trying to cause an error (e.g., set invalid parameter or command).")
    #     # Example: Try to set an op mode that's not supported or in a wrong state
    #     # node.set_op_mode(99) # Assuming 99 is not a valid mode
    #     # time.sleep(2) # Wait for EMCY if any
    #
    #     # Test fault reset
    #     # if node.state == 'FAULT':
    #     #    node.fault_reset()
    #
    # except Exception as e:
    #     print(f"Diagnostics example error: {e.__class__.__name__}: {e}")
    #     import traceback
    #     traceback.print_exc()
    # finally:
    #     if node: node.nmt_reset_communication()
    #     if network: network.disconnect()
    #     print("Diagnostics example finished.")
    pass
