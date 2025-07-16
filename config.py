#!/usr/bin/env python

import dbus
import subprocess
import sys
import logging
import random
from gi.repository import GLib
from dbus.mainloop.glib import DBusGMainLoop

# Adjust the path for velib_python if necessary based on your Cerbo GX setup
sys.path.insert(1, "/opt/victronenergy/dbus-systemcalc-py/ext/velib_python")
from settingsdevice import SettingsDevice
from ve_utils import wrap_dbus_value

# --- Logging Configuration ---
# Log to console for user interaction
logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')

# --- D-Bus Constants ---
DBUS_SETTINGS_SERVICE = "com.victronenergy.settings"
DBUS_SETTINGS_BASE_PATH = "/Settings/MQTTSwitches" # Updated base path

# --- Helper for dbus commands (for adding/removing settings) ---
def _run_dbus_command(command_args):
    """
    Helper function to execute dbus commands for adding/removing settings.
    This mimics the behavior of DbusSettingsResources.txt using the 'dbus' command.
    """
    try:
        # Using check_output to capture stdout and raise CalledProcessError for non-zero exit codes
        result = subprocess.check_output(command_args, stderr=subprocess.STDOUT, text=True)
        return True, result.strip()
    except subprocess.CalledProcessError as e:
        logging.error(f"D-Bus command failed: {' '.join(e.cmd)}")
        logging.error(f"Return Code: {e.returncode}")
        logging.error(f"Output: {e.output.strip()}")
        return False, e.output.strip()
    except Exception as e:
        logging.error(f"An unexpected error occurred while executing D-Bus command: {e}")
        return False, str(e)

# --- Functions for managing D-Bus settings (create/remove using 'dbus' command) ---

def add_dbus_setting(path, default_value, data_type_char):
    """
    Adds a new D-Bus setting. Uses 'dbus -y' command.
    data_type_char: 's' for string, 'i' for int, 'd' for double.
    """
    if data_type_char not in ['s', 'i', 'd']:
        logging.error(f"Invalid data type character '{data_type_char}'. Use 's', 'i', or 'd'.")
        return False

    default_val_str = ""
    if data_type_char == 's':
        # Ensure default string values are correctly quoted and escaped for the D-Bus array string
        # If default_value is None, it should become '""'
        if default_value is None:
            default_val_for_json = '""'
        else:
            default_val_for_json = f'"{str(default_value).replace('"', '\\"')}"'
        default_val_str = f'"default":{default_val_for_json}'
    elif data_type_char == 'i':
        default_val_str = f'"default":{int(default_value) if default_value is not None else 0}'
    elif data_type_char == 'd':
        default_val_str = f'"default":{float(default_value) if default_value is not None else 0.0}'
    
    # Construct the struct string for the D-Bus array
    settings_arg_struct = f'{{ "path":"{path}", {default_val_str} }}'
    
    command = [
        "dbus", "-y", # -y automatically confirms "yes"
        DBUS_SETTINGS_SERVICE,
        "/",  # Object path for AddSettings method
        "AddSettings",
        f"%[ {settings_arg_struct} ]" # Pass the array of structs as a single string argument
    ]
    logging.info(f"Attempting to add setting: {path} (Default: {default_value}, Type: {data_type_char})")
    success, _ = _run_dbus_command(command)
    if success:
        logging.info(f"Setting '{path}' added successfully.")
    else:
        logging.error(f"Failed to add setting '{path}'.")
    return success

def remove_dbus_setting(path):
    """
    Removes a D-Bus setting. Uses 'dbus -y' command.
    """
    command = [
        "dbus", "-y", # -y automatically confirms "yes"
        DBUS_SETTINGS_SERVICE,
        "/",  # Object path for RemoveSettings method
        "RemoveSettings",
        f"%[ \"{path}\" ]" # Pass the array of strings as a single string argument
    ]
    logging.info(f"Attempting to remove setting: {path}")
    success, _ = _run_dbus_command(command)
    if success:
        logging.info(f"Setting '{path}' removed successfully.")
    else:
        logging.error(f"Failed to remove setting '{path}'.")
    return success

# --- Main Configuration Class ---
class ServiceConfig:
    def __init__(self):
        # Initialize D-Bus connection
        self.bus = dbus.SystemBus()

        # Define all settings with their default values and types for SettingsDevice
        # We'll use this for getting and setting existing values,
        # and for initial creation (though AddSettings is used explicitly for new paths)
        self.settings_list = {
            'LogLevel': [f'{DBUS_SETTINGS_BASE_PATH}/LogLevel', 'INFO', 0, 0],
            'MqttBrokerAddress': [f'{DBUS_SETTINGS_BASE_PATH}/Mqtt/BrokerAddress', 'localhost', 0, 0],
            'MqttBrokerPort': [f'{DBUS_SETTINGS_BASE_PATH}/Mqtt/BrokerPort', 1883, 0, 0],
            'MqttUsername': [f'{DBUS_SETTINGS_BASE_PATH}/Mqtt/Username', '', 0, 0],
            'MqttPassword': [f'{DBUS_SETTINGS_BASE_PATH}/Mqtt/Password', '', 0, 0],
            'NumberOfDevices': [f'{DBUS_SETTINGS_BASE_PATH}/NumberOfDevices', 1, 0, 100], # Max 100 devices

            # Dynamic settings for devices and switches will be handled
            # by adding/removing specific paths as needed.
            # SettingsDevice primarily tracks the settings it manages itself.
            # For this script, we use it to initialize and read/write values.
        }

        # Initialize SettingsDevice
        self.dbus_settings = SettingsDevice(bus=self.bus, supportedSettings=self.settings_list,
                                            timeout=10, eventCallback=None)
        
        # Cache for dynamic device/switch settings for quick access
        self.device_settings_cache = {}

        # Ensure base path exists for initial settings
        self._ensure_base_settings_exist()
        
    def _ensure_base_settings_exist(self):
        """
        Ensures the core settings for the service exist in D-Bus.
        If they don't, it adds them using dbus command.
        """
        logging.info("Checking for existence of core service settings...")
        for key, details in self.settings_list.items():
            path = details[0]
            default_value = details[1]
            
            # Determine data type char for AddSettings
            if isinstance(default_value, str):
                data_type_char = 's'
            elif isinstance(default_value, int):
                data_type_char = 'i'
            elif isinstance(default_value, float):
                data_type_char = 'd'
            else:
                logging.warning(f"Unsupported type for setting {path}. Skipping initial check.")
                continue

            try:
                # Attempt to get the value to check if it exists
                obj = self.bus.get_object(DBUS_SETTINGS_SERVICE, path)
                obj.GetValue() # If this doesn't raise an exception, it exists
            except dbus.exceptions.DBusException:
                # Setting does not exist, so add it
                logging.warning(f"Setting '{path}' does not exist. Adding it with default value.")
                add_dbus_setting(path, default_value, data_type_char)
            except Exception as e:
                logging.error(f"Error checking setting '{path}': {e}")


    def get_setting(self, setting_key):
        """
        Retrieves a global setting from D-Bus.
        """
        try:
            # Use SettingsDevice for simple access to its managed settings
            return self.dbus_settings[setting_key]
        except KeyError:
            logging.error(f"Setting key '{setting_key}' not found in internal list. Trying direct D-Bus call.")
            # Fallback to direct D-Bus object if not in SettingsDevice (e.g. dynamic paths)
            try:
                obj = self.bus.get_object(DBUS_SETTINGS_SERVICE, setting_key)
                return obj.GetValue()
            except dbus.exceptions.DBusException as e:
                logging.error(f"Failed to get D-Bus setting directly for '{setting_key}': {e}")
                return None
        except Exception as e:
            logging.error(f"An unexpected error occurred getting setting '{setting_key}': {e}")
            return None

    def set_setting(self, setting_key, value):
        """
        Sets a global setting in D-Bus.
        """
        try:
            # Use SettingsDevice for simple access to its managed settings
            self.dbus_settings[setting_key] = value
            logging.info(f"Setting '{setting_key}' updated to '{value}'.")
            return True
        except KeyError:
            logging.error(f"Setting key '{setting_key}' not found in internal list. Cannot set directly via SettingsDevice.")
            # For dynamic paths (devices/switches), we need to set directly via BusItem.SetValue()
            # This is complex as it requires knowing the exact D-Bus path and data type for SetValue.
            # For this script, we'll keep the direct set via `dbus` command for modifications.
            logging.warning("Please ensure the setting is already created before attempting to set dynamic paths.")
            return False
        except Exception as e:
            logging.error(f"An unexpected error occurred setting '{setting_key}': {e}")
            return False

    def get_dynamic_setting_path(self, *components):
        """Constructs a D-Bus path for dynamic settings (devices/switches)."""
        return f"{DBUS_SETTINGS_BASE_PATH}/{'/'.join(map(str, components))}"

    def get_dynamic_setting(self, path):
        """
        Gets a dynamic setting (device/switch related) from D-Bus.
        Uses direct dbus library GetValue.
        """
        try:
            obj = self.bus.get_object(DBUS_SETTINGS_SERVICE, path)
            value = obj.GetValue()
            logging.debug(f"Retrieved dynamic setting '{path}': {value}")
            return value
        except dbus.exceptions.DBusException as e:
            # Suppress specific errors that indicate the setting simply doesn't exist
            error_message = str(e)
            if "Does not exist" in error_message or "UnknownObject" in error_message:
                logging.debug(f"Dynamic setting '{path}' does not exist (expected for new settings).")
            else:
                # Log other D-Bus exceptions as errors
                logging.error(f"Error getting dynamic D-Bus setting '{path}': {e}")
            return None
        except Exception as e:
            logging.error(f"An unexpected error occurred getting dynamic setting '{path}': {e}")
            return None

    def set_dynamic_setting(self, path, value, data_type_str):
        """
        Sets a dynamic setting (device/switch related) in D-Bus.
        Uses direct dbus library to call SetValue on the setting's object path.
        data_type_str: 'string', 'int', 'float', 'bool' - used for logging/type hint if needed,
                                                     but dbus library handles marshaling.
        """
        try:
            # Get the D-Bus object for the specific setting path
            obj = self.bus.get_object(DBUS_SETTINGS_SERVICE, path)
            
            # Call the SetValue method on that object
            # The dbus library generally handles Python types to D-Bus types automatically.
            # No need for explicit variant wrapping unless dealing with ambiguous types,
            # but for string, int, float, bool it should be straightforward.
            obj.SetValue(value)
            logging.info(f"Successfully set dynamic setting '{path}' to '{value}'.")
            return True
        except dbus.exceptions.DBusException as e:
            logging.error(f"Failed to set dynamic D-Bus setting '{path}' to '{value}': {e}")
            return False
        except Exception as e:
            logging.error(f"An unexpected error occurred setting dynamic setting '{path}': {e}")
            return False
    
    def _get_next_device_instance(self):
        """Finds the next available device instance number starting from 100."""
        
        used_instance_numbers = set()
        
        for conceptual_idx in range(100): # Scan potential conceptual device indices (0-99)
            path = self.get_dynamic_setting_path(f"Device/{conceptual_idx}/Instance")
            instance_val = self.get_dynamic_setting(path)
            if instance_val is not None and isinstance(instance_val, int):
                used_instance_numbers.add(instance_val)
        
        # Now find the first available instance number starting from 100
        for candidate_instance in range(100, 200): # Iterate through actual instance numbers (100, 101, ...)
            if candidate_instance not in used_instance_numbers:
                logging.info(f"Found next available instance number: {candidate_instance}")
                return candidate_instance
        
        logging.error("No available device instance numbers found in the range 100-199. Max limit reached or too many existing uncleaned settings.")
        return None

    def _generate_random_serial(self):
        """Generates a random 16-digit serial number string."""
        return ''.join(random.choices('0123456789', k=16))


    def configure_global_settings(self):
        """Prompts for and sets global settings."""
        logging.info("\n--- Configuring Global Settings ---")

        # Logging Level
        log_level = input(f"Enter Logging Level (INFO, DEBUG, WARNING, ERROR, CRITICAL) [{self.get_setting('LogLevel')}]: ")
        if log_level:
            self.set_setting('LogLevel', log_level.upper())

        # MQTT Broker Address
        mqtt_broker_address = input(f"Enter MQTT Broker Address [{self.get_setting('MqttBrokerAddress')}]: ")
        if mqtt_broker_address:
            self.set_setting('MqttBrokerAddress', mqtt_broker_address)

        # MQTT Broker Port
        mqtt_broker_port_str = input(f"Enter MQTT Broker Port Number [{self.get_setting('MqttBrokerPort')}]: ")
        if mqtt_broker_port_str:
            try:
                mqtt_broker_port = int(mqtt_broker_port_str)
                self.set_setting('MqttBrokerPort', mqtt_broker_port)
            except ValueError:
                logging.error("Invalid port number. Must be an integer.")

        # MQTT Username
        mqtt_username = input(f"Enter MQTT Username (leave blank for no username) [{self.get_setting('MqttUsername')}]: ")
        # Allow clearing by entering empty string
        self.set_setting('MqttUsername', mqtt_username)

        # MQTT Password
        mqtt_password = input(f"Enter MQTT Password (leave blank for no password) [{self.get_setting('MqttPassword')}]: ")
        # Allow clearing by entering empty string
        self.set_setting('MqttPassword', mqtt_password)
        
        # Number of Devices
        num_devices_str = input(f"Enter Number of Devices (1-100) [{self.get_setting('NumberOfDevices')}]: ")
        if num_devices_str:
            try:
                num_devices = int(num_devices_str)
                if 1 <= num_devices <= 100:
                    self.set_setting('NumberOfDevices', num_devices)
                else:
                    logging.error("Number of devices must be between 1 and 100.")
            except ValueError:
                logging.error("Invalid number of devices. Must be an integer.")


    def configure_devices_and_switches(self):
        """Prompts for and configures device and switch settings."""
        num_devices = self.get_setting('NumberOfDevices')
        if num_devices is None:
            logging.error("Could not retrieve number of devices. Cannot configure devices/switches.")
            return

        logging.info(f"\n--- Configuring {num_devices} Devices ---")
        
        # Keep track of device instances that are actually configured
        configured_device_instances = []

        for i in range(num_devices):
            logging.info(f"\n--- Device {i + 1}/{num_devices} ---")
            
            # Auto-assign instance, or retrieve if already set for this conceptual device index
            device_instance_path = self.get_dynamic_setting_path(f"Device/{i}/Instance")
            current_instance = self.get_dynamic_setting(device_instance_path)

            if current_instance is None:
                # Need to find a new instance for this device
                instance = self._get_next_device_instance()
                if instance is None:
                    logging.error("Failed to get a new device instance. Aborting device configuration.")
                    break
                # Add the instance setting using the improved add_dbus_setting
                add_dbus_setting(device_instance_path, instance, 'i')
                self.set_dynamic_setting(device_instance_path, instance, 'int') # Set the value
            else:
                instance = current_instance
            
            configured_device_instances.append(instance)

            # Get or set device-specific settings
            device_paths = {
                'NumberOfSwitches': self.get_dynamic_setting_path(f"Device/{i}/NumberOfSwitches"),
                'Instance': self.get_dynamic_setting_path(f"Device/{i}/Instance"), # Already handled above, but for completeness
                'CustomName': self.get_dynamic_setting_path(f"Device/{i}/CustomName"),
                'MqttOnPayload': self.get_dynamic_setting_path(f"Device/{i}/MqttOnPayload"),
                'MqttOffPayload': self.get_dynamic_setting_path(f"Device/{i}/MqttOffPayload"),
                'SerialNumber': self.get_dynamic_setting_path(f"Device/{i}/SerialNumber"), # New serial number path
            }
            
            # Ensure these dynamic settings exist before trying to get/set
            # Add them if they don't, with reasonable defaults using the improved add_dbus_setting
            if self.get_dynamic_setting(device_paths['NumberOfSwitches']) is None:
                add_dbus_setting(device_paths['NumberOfSwitches'], 1, 'i')
            if self.get_dynamic_setting(device_paths['CustomName']) is None:
                add_dbus_setting(device_paths['CustomName'], f"Device {i+1}", 's')
            if self.get_dynamic_setting(device_paths['MqttOnPayload']) is None:
                add_dbus_setting(device_paths['MqttOnPayload'], "ON", 's')
            if self.get_dynamic_setting(device_paths['MqttOffPayload']) is None:
                add_dbus_setting(device_paths['MqttOffPayload'], "OFF", 's')
            
            # Handle Serial Number
            serial_number_path = device_paths['SerialNumber']
            current_serial = self.get_dynamic_setting(serial_number_path)
            
            if current_serial is None:
                # Serial number doesn't exist, generate a new one
                new_serial = self._generate_random_serial()
                add_dbus_setting(serial_number_path, new_serial, 's')
                self.set_dynamic_setting(serial_number_path, new_serial, 'string')
                logging.info(f"Generated new serial number for Device {instance}: {new_serial}")
                current_serial = new_serial # Update current_serial for prompt
            
            serial_input = input(f"Device {instance} - Enter Serial Number (16 digits) [{current_serial}]: ")
            if serial_input:
                if len(serial_input) == 16 and serial_input.isdigit():
                    self.set_dynamic_setting(serial_number_path, serial_input, 'string')
                else:
                    logging.error("Invalid serial number. Must be 16 digits.")

            # Number of Switches
            num_switches_path = device_paths['NumberOfSwitches']
            num_switches_current = self.get_dynamic_setting(num_switches_path)
            num_switches_str = input(f"Device {instance} - Enter Number of Switches (1-100) [{num_switches_current}]: ")
            if num_switches_str:
                try:
                    num_switches = int(num_switches_str)
                    if 1 <= num_switches <= 100:
                        self.set_dynamic_setting(num_switches_path, num_switches, 'int')
                    else:
                        logging.error("Number of switches must be between 1 and 100.")
                        num_switches = num_switches_current # Revert to current if invalid input
                except ValueError:
                    logging.error("Invalid number of switches. Must be an integer.")
            else:
                num_switches = num_switches_current

            # Custom Name
            custom_name_path = device_paths['CustomName']
            custom_name_current = self.get_dynamic_setting(custom_name_path)
            custom_name = input(f"Device {instance} - Enter Custom Name [{custom_name_current}]: ")
            if custom_name:
                self.set_dynamic_setting(custom_name_path, custom_name, 'string')

            # MQTT ON Payload
            mqtt_on_payload_path = device_paths['MqttOnPayload']
            mqtt_on_payload_current = self.get_dynamic_setting(mqtt_on_payload_path)
            mqtt_on_payload = input(f"Device {instance} - Enter MQTT ON Payload [{mqtt_on_payload_current}]: ")
            if mqtt_on_payload:
                self.set_dynamic_setting(mqtt_on_payload_path, mqtt_on_payload, 'string')

            # MQTT OFF Payload
            mqtt_off_payload_path = device_paths['MqttOffPayload']
            mqtt_off_payload_current = self.get_dynamic_setting(mqtt_off_payload_path)
            mqtt_off_payload = input(f"Device {instance} - Enter MQTT OFF Payload [{mqtt_off_payload_current}]: ")
            if mqtt_off_payload:
                self.set_dynamic_setting(mqtt_off_payload_path, mqtt_off_payload, 'string')

            # Configure Switches for this device
            for j in range(num_switches):
                logging.info(f"  --- Device {instance} - Switch {j + 1}/{num_switches} ---")
                
                switch_paths = {
                    'CustomName': self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/CustomName"),
                    'Group': self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/Group"),
                    'StateTopic': self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/StateTopic"),
                    'CommandTopic': self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/CommandTopic"),
                }

                # Ensure these dynamic switch settings exist before trying to get/set
                # using the improved add_dbus_setting
                if self.get_dynamic_setting(switch_paths['CustomName']) is None:
                    add_dbus_setting(switch_paths['CustomName'], f"Switch {j+1}", 's')
                if self.get_dynamic_setting(switch_paths['Group']) is None:
                    add_dbus_setting(switch_paths['Group'], f"Group {i+1}", 's')
                if self.get_dynamic_setting(switch_paths['StateTopic']) is None:
                    add_dbus_setting(switch_paths['StateTopic'], f"mqtt_switches/device_{instance}/switch_{j+1}/state", 's')
                if self.get_dynamic_setting(switch_paths['CommandTopic']) is None:
                    add_dbus_setting(switch_paths['CommandTopic'], f"mqtt_switches/device_{instance}/switch_{j+1}/command", 's')


                # Custom Name
                switch_custom_name_path = switch_paths['CustomName']
                switch_custom_name_current = self.get_dynamic_setting(switch_custom_name_path)
                switch_custom_name = input(f"    Switch {j+1} - Enter Custom Name [{switch_custom_name_current}]: ")
                if switch_custom_name:
                    self.set_dynamic_setting(switch_custom_name_path, switch_custom_name, 'string')

                # Group
                group_path = switch_paths['Group']
                group_current = self.get_dynamic_setting(group_path)
                group = input(f"    Switch {j+1} - Enter Group [{group_current}]: ")
                if group:
                    self.set_dynamic_setting(group_path, group, 'string')

                # State Topic
                state_topic_path = switch_paths['StateTopic']
                state_topic_current = self.get_dynamic_setting(state_topic_path)
                state_topic = input(f"    Switch {j+1} - Enter State Topic [{state_topic_current}]: ")
                if state_topic:
                    self.set_dynamic_setting(state_topic_path, state_topic, 'string')

                # Command Topic
                command_topic_path = switch_paths['CommandTopic']
                command_topic_current = self.get_dynamic_setting(command_topic_path)
                command_topic = input(f"    Switch {j+1} - Enter Command Topic [{command_topic_current}]: ")
                if command_topic:
                    self.set_dynamic_setting(command_topic_path, command_topic, 'string')
        
        # Cleanup: Remove settings for devices that are no longer configured
        # This requires knowing all existing device instances and comparing them with configured_device_instances
        logging.info("Cleaning up old device and switch settings...")
        current_max_devices = 100 # Assuming max 100 devices based on instance check logic
        for i in range(current_max_devices):
            device_instance_path = self.get_dynamic_setting_path(f"Device/{i}/Instance")
            instance_val = self.get_dynamic_setting(device_instance_path)
            
            # If an instance exists at this index but is not in our currently configured list
            if instance_val is not None and instance_val not in configured_device_instances:
                logging.info(f"Found unconfigured device index {i} (Instance {instance_val}). Removing its settings.")
                # Remove all settings associated with this device index
                self.remove_device_settings(i) # Use index i to remove the specific path structure


    def remove_device_settings(self, device_idx):
        """Removes all settings for a given device index and its switches."""
        base_device_path = self.get_dynamic_setting_path(f"Device/{device_idx}")
        logging.info(f"Removing all settings under {base_device_path}...")

        # Paths to remove directly
        paths_to_remove = [
            self.get_dynamic_setting_path(f"Device/{device_idx}/NumberOfSwitches"),
            self.get_dynamic_setting_path(f"Device/{device_idx}/Instance"),
            self.get_dynamic_setting_path(f"Device/{device_idx}/CustomName"),
            self.get_dynamic_setting_path(f"Device/{device_idx}/MqttOnPayload"),
            self.get_dynamic_setting_path(f"Device/{device_idx}/MqttOffPayload"),
            self.get_dynamic_setting_path(f"Device/{device_idx}/SerialNumber"), # New serial number path
        ]

        # Get existing number of switches for this device before removing
        num_switches_path = self.get_dynamic_setting_path(f"Device/{device_idx}/NumberOfSwitches")
        num_switches = self.get_dynamic_setting(num_switches_path)
        if num_switches is None:
            num_switches = 1 # Assume at least 1 to try and clean up if setting is missing

        for j in range(num_switches):
            paths_to_remove.extend([
                self.get_dynamic_setting_path(f"Device/{device_idx}/Switch/{j}/CustomName"),
                self.get_dynamic_setting_path(f"Device/{device_idx}/Switch/{j}/Group"),
                self.get_dynamic_setting_path(f"Device/{device_idx}/Switch/{j}/StateTopic"),
                self.get_dynamic_setting_path(f"Device/{device_idx}/Switch/{j}/CommandTopic"),
            ])
        
        # Execute removal for all collected paths
        for path in paths_to_remove:
            remove_dbus_setting(path) # Using the improved remove_dbus_setting
        
        logging.info(f"Finished removing settings for device index {device_idx}.")


    def view_all_settings(self):
        """Displays all configured settings."""
        logging.info("\n--- Current Configuration ---")
        
        # Global Settings
        logging.info("\nGlobal Settings:")
        for key in self.settings_list:
            value = self.get_setting(key)
            logging.info(f"  {key}: {value}")

        # Device and Switch Settings
        num_devices = self.get_setting('NumberOfDevices')
        if num_devices is None:
            logging.error("Could not retrieve number of devices.")
            return

        logging.info(f"\nDevice Settings (Configured for {num_devices} devices):")
        # Iterate up to the configured number of devices
        for i in range(num_devices):
            instance_path = self.get_dynamic_setting_path(f"Device/{i}/Instance")
            instance = self.get_dynamic_setting(instance_path)
            
            if instance is None: 
                logging.info(f"  Device {i+1}: No instance found (may not be fully configured or deleted).")
                continue

            num_switches_path = self.get_dynamic_setting_path(f"Device/{i}/NumberOfSwitches")
            num_switches = self.get_dynamic_setting(num_switches_path)
            if num_switches is None:
                num_switches = 0 # Default to 0 if not set

            custom_name_path = self.get_dynamic_setting_path(f"Device/{i}/CustomName")
            custom_name = self.get_dynamic_setting(custom_name_path)
            
            mqtt_on_payload_path = self.get_dynamic_setting_path(f"Device/{i}/MqttOnPayload")
            mqtt_on_payload = self.get_dynamic_setting(mqtt_on_payload_path)

            mqtt_off_payload_path = self.get_dynamic_setting_path(f"Device/{i}/MqttOffPayload")
            mqtt_off_payload = self.get_dynamic_setting(mqtt_off_payload_path)

            serial_number_path = self.get_dynamic_setting_path(f"Device/{i}/SerialNumber")
            serial_number = self.get_dynamic_setting(serial_number_path)


            logging.info(f"  Device {i+1} (Instance: {instance}, Name: '{custom_name}')")
            logging.info(f"    Serial Number: {serial_number}")
            logging.info(f"    Number of Switches: {num_switches}")
            logging.info(f"    MQTT ON Payload: {mqtt_on_payload}")
            logging.info(f"    MQTT OFF Payload: {mqtt_off_payload}")

            for j in range(num_switches):
                switch_custom_name_path = self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/CustomName")
                switch_custom_name = self.get_dynamic_setting(switch_custom_name_path)

                group_path = self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/Group")
                group = self.get_dynamic_setting(group_path)

                state_topic_path = self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/StateTopic")
                state_topic = self.get_dynamic_setting(state_topic_path)

                command_topic_path = self.get_dynamic_setting_path(f"Device/{i}/Switch/{j}/CommandTopic")
                command_topic = self.get_dynamic_setting(command_topic_path)

                logging.info(f"    Switch {j+1} (Name: '{switch_custom_name}')")
                logging.info(f"      Group: {group}")
                logging.info(f"      State Topic: {state_topic}")
                logging.info(f"      Command Topic: {command_topic}")

    def delete_all_settings(self):
        """Deletes all settings associated with the service."""
        confirm = input("Are you absolutely sure you want to delete ALL settings for MQTTSwitches? (type 'yes' to confirm): ").lower()
        if confirm != 'yes':
            logging.info("Deletion cancelled.")
            return

        logging.info("\n--- Deleting All MQTTSwitches Settings ---")
        
        # Delete global settings
        for key, details in self.settings_list.items():
            remove_dbus_setting(details[0])

        # Delete dynamic device and switch settings
        # This requires iterating through possible device and switch indices
        logging.info("Searching for and deleting dynamic device/switch settings...")
        for i in range(100): # Check up to 100 potential device indices
            device_base_path = self.get_dynamic_setting_path(f"Device/{i}")
            # Check if any part of the device path exists before attempting extensive removal
            if self.get_dynamic_setting(self.get_dynamic_setting_path(f"Device/{i}/Instance")) is not None:
                logging.info(f"Found device settings at index {i}. Deleting...")
                self.remove_device_settings(i) # Reuse the per-device removal logic
        
        logging.info("All MQTTSwitches settings deletion process complete.")


def main():
    # Have a mainloop, so we can send/receive asynchronous calls to and from dbus
    DBusGMainLoop(set_as_default=True)

    config_manager = ServiceConfig()

    while True:
        print("\n--- MQTTSwitches Configuration Menu ---")
        print("1. Configure Global Settings")
        print("2. Configure Devices and Switches")
        print("3. View Current Configuration")
        print("4. Delete All MQTTSwitches Settings")
        print("5. Exit")

        choice = input("Enter your choice (1-5): ")

        if choice == '1':
            config_manager.configure_global_settings()
        elif choice == '2':
            config_manager.configure_devices_and_switches()
        elif choice == '3':
            config_manager.view_all_settings()
        elif choice == '4':
            config_manager.delete_all_settings()
        elif choice == '5':
            print("Exiting configuration tool.")
            sys.exit(0)
        else:
            print("Invalid choice. Please enter a number between 1 and 5.")

if __name__ == "__main__":
    main()
