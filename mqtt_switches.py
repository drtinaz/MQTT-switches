#!/usr/bin/env python3

from gi.repository import GLib
import logging
import sys
import os
import random
import subprocess
import time
import paho.mqtt.client as mqtt
import threading

# --- Adjust the path for velib_python if necessary based on your Cerbo GX setup ---
sys.path.insert(1, "/opt/victronenergy/dbus-systemcalc-py/ext/velib_python")
try:
    from vedbus import VeDbusService
except ImportError:
    logging.critical("Cannot find vedbus library. Please ensure it's in the correct path.")
    sys.exit(1)

# --- Import ServiceConfig from the config.py script ---
# Assuming config.py is in the same directory or accessible via sys.path
try:
    # Attempt to import from the sibling config.py file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_py_path = os.path.join(script_dir, 'config.py')
    
    # Temporarily add script_dir to sys.path to allow direct import of config
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    
    # Import the ServiceConfig and DBUS_SETTINGS_BASE_PATH
    from config import ServiceConfig, DBUS_SETTINGS_BASE_PATH
    
except ImportError:
    logging.critical("Cannot find config.py or necessary components. Please ensure it's in the correct path.")
    sys.exit(1)
finally:
    # Clean up sys.path if it was modified
    if 'script_dir' in locals() and script_dir in sys.path:
        sys.path.remove(script_dir)


# --- Logging Configuration ---
# Get the root logger instance
logger = logging.getLogger()

# Remove all existing handlers from the root logger
for handler in logger.handlers[:]:
    logger.removeHandler(handler)

# Get the directory of the currently running script
script_dir = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(script_dir, 'log')

# Create a FileHandler to write logs to the 'log' file
file_handler = logging.FileHandler(log_file_path)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# Default log level, will be overridden by D-Bus setting
logger.setLevel(logging.INFO)

# --- No longer using configparser or CONFIG_FILE_PATH ---

class DbusMyTestSwitch(VeDbusService):

    def __init__(self, service_name, device_data, output_configs, mqtt_config):
        # Paths are added first, then the service is registered.
        super().__init__(service_name, register=False)

        # Store device and output config data
        self.device_data = device_data
        self.output_configs = output_configs
        self.dbus_device_idx = device_data['DeviceIndex'] # This is the 0-based conceptual index

        # General device settings
        self.add_path('/Mgmt/ProcessName', 'dbus-victron-virtual')
        self.add_path('/Mgmt/ProcessVersion', '0.1.16')
        self.add_path('/Mgmt/Connection', 'Virtual')
        
        self.add_path('/DeviceInstance', self.device_data['DeviceInstance'])
        self.add_path('/ProductId', 49257)
        self.add_path('/ProductName', 'Virtual switch')
        # CustomName is read-only here, managed by config.py
        self.add_path('/CustomName', self.device_data['CustomName']) 
        
        self.add_path('/Serial', self.device_data['Serial'])
        
        self.add_path('/State', 256) # Default state, check if this should be dynamically set
        
        self.add_path('/FirmwareVersion', 0)
        self.add_path('/HardwareVersion', 0)
        self.add_path('/Connected', 1)

        # MQTT specific members
        self.mqtt_client = None
        self.mqtt_config = mqtt_config
        self.dbus_path_to_state_topic_map = {}
        self.dbus_path_to_command_topic_map = {}
        
        self.mqtt_on_payload = self.device_data['MqttOnPayload']
        self.mqtt_off_payload = self.device_data['MqttOffPayload']

        # Loop through the outputs and add their D-Bus paths
        for output_data in output_configs:
            self.add_output(output_data)

        # Initialize and connect the MQTT client
        self.setup_mqtt_client()
        
        # Register the service on the D-Bus AFTER all paths have been added
        self.register()
        logger.info(f"Service '{service_name}' for device '{self.device_data['CustomName']}' (Instance: {self.device_data['DeviceInstance']}) registered on D-Bus.")

    def add_output(self, output_data):
        """
        Adds a single switchable output and its settings to the D-Bus service,
        and stores MQTT topic mappings.
        """
        # output_data['index'] here is the 0-based D-Bus index for the switch
        output_prefix = f'/SwitchableOutput/output_{output_data["index"]}'
        
        # Store topic mappings for later use
        state_topic = output_data.get('MqttStateTopic')
        command_topic = output_data.get('MqttCommandTopic')
        dbus_state_path = f'{output_prefix}/State'

        if state_topic and command_topic:
            self.dbus_path_to_state_topic_map[dbus_state_path] = state_topic
            self.dbus_path_to_command_topic_map[dbus_state_path] = command_topic

        self.add_path(f'{output_prefix}/Name', output_data['name'])
        self.add_path(f'{output_prefix}/Status', 0)

        # Add the State path, which will be writable.
        self.add_path(dbus_state_path, 0, writeable=True, onchangecallback=self.handle_dbus_change)

        settings_prefix = f'{output_prefix}/Settings'
        # These are read-only from here, managed by config.py
        self.add_path(f'{settings_prefix}/CustomName', output_data['custom_name'])
        self.add_path(f'{settings_prefix}/Group', output_data['group'])
        self.add_path(f'{settings_prefix}/Type', 1) # Not writable from here
        self.add_path(f'{settings_prefix}/ValidTypes', 7) # Not writable from here

    def setup_mqtt_client(self):
        """
        Initializes and starts the MQTT client.
        """
        self.mqtt_client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self['/Serial']
        )
        
        if self.mqtt_config.get('Username'):
            self.mqtt_client.username_pw_set(
                self.mqtt_config.get('Username'),
                self.mqtt_config.get('Password')
            )
            
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_publish = self.on_mqtt_publish
        
        try:
            self.mqtt_client.connect(
                self.mqtt_config.get('BrokerAddress'),
                self.mqtt_config.get('Port', 1883), # Ensure port is int
                60
            )
            self.mqtt_client.loop_start()
            logger.debug("MQTT client started.")
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc, properties):
        """
        MQTT callback for when the client connects to the broker.
        Subscribes only to state topics.
        """
        if rc == 0:
            logger.debug("Connected to MQTT Broker!")
            state_topics = list(self.dbus_path_to_state_topic_map.values())
            for topic in state_topics:
                client.subscribe(topic)
                logger.debug(f"Subscribed to MQTT state topic: {topic}")
        else:
            logger.error(f"Failed to connect to MQTT broker, return code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """
        MQTT callback for when a message is received on a subscribed topic.
        Only accepts payloads as defined in the D-Bus config.
        """
        try:
            payload = msg.payload.decode().strip().lower()
            topic = msg.topic
            logger.debug(f"Received MQTT message on topic '{topic}': {payload}")

            new_state = None
            if payload == self.mqtt_on_payload.lower():
                new_state = 1
            elif payload == self.mqtt_off_payload.lower():
                new_state = 0
            else:
                logger.warning(f"Invalid MQTT payload received for topic '{topic}': '{payload}'. Expected '{self.mqtt_on_payload}' or '{self.mqtt_off_payload}'.")
                return
            
            dbus_path = next((k for k, v in self.dbus_path_to_state_topic_map.items() if v == topic), None)
            
            if dbus_path:
                if self[dbus_path] == new_state:
                    logger.debug(f"D-Bus state is already {new_state}, ignoring redundant MQTT message.")
                    return
                
                # Use GLib.idle_add to schedule the D-Bus update in the main thread
                GLib.idle_add(self.update_dbus_from_mqtt, dbus_path, new_state)
            
        except (ValueError, KeyError) as e:
            logger.error(f"Error processing MQTT message: {e}")
    
    def on_mqtt_publish(self, client, userdata, mid, reason_code, properties):
        """
        MQTT callback for when a publish request has been sent.
        """
        logger.debug(f"Publish message with mid: {mid} acknowledged by client.")

    def handle_dbus_change(self, path, value):
        """
        Callback function to handle changes to D-Bus paths.
        This is triggered when a D-Bus client requests a change.
        Only publishes to MQTT if a state path changes.
        Other changes are handled by config.py, this script just reads them.
        """
        if "/State" in path:
            logger.debug(f"D-Bus change handler triggered for {path} with value {value}")
            if value not in [0, 1]:
                logger.warning(f"Invalid D-Bus state value received: {value}. Expected 0 or 1.")
                return False
            self.publish_mqtt_command(path, value)
            return True
        
        logger.debug(f"D-Bus change for path: {path} is not handled for MQTT publish or config saving by mqtt_switches.py. Persistence is handled by config.py.")
        return False # Return False if this script doesn't handle the persistence directly

    def publish_mqtt_command(self, path, value):
        """
        Centralized and robust method to publish a command to MQTT.
        """
        if not self.mqtt_client or not self.mqtt_client.is_connected():
            logger.warning("MQTT client is not connected. Cannot publish.")
            return

        if path not in self.dbus_path_to_command_topic_map:
            logger.warning(f"No command topic mapped for D-Bus path: {path}")
            return

        try:
            command_topic = self.dbus_path_to_command_topic_map[path]
            mqtt_payload = self.mqtt_on_payload if value == 1 else self.mqtt_off_payload

            (rc, mid) = self.mqtt_client.publish(command_topic, mqtt_payload, retain=False)
            
            if rc == mqtt.MQTT_ERR_SUCCESS:
                logger.debug(f"Publish request for '{path}' sent to command topic '{command_topic}'. mid: {mid}")
            else:
                logger.error(f"Failed to publish to '{command_topic}', return code: {rc}")
        except Exception as e:
            logger.error(f"Error during MQTT publish: {e}")
            
    def update_dbus_from_mqtt(self, path, value):
        """
        A centralized method to handle MQTT-initiated state changes to D-Bus.
        """
        self[path] = value
        logger.debug(f"Successfully changed '{path}' to {value} from source: mqtt")
        
        return False # Return False for GLib.idle_add to run only once

def run_device_service(conceptual_device_index_str):
    """
    Main function for a single D-Bus service process.
    `conceptual_device_index_str` is the 1-based index (e.g., "1", "2") passed from the launcher.
    """
    from dbus.mainloop.glib import DBusGMainLoop
    DBusGMainLoop(set_as_default=True) # Ensure main loop is set for child processes
    
    # Calculate the 0-based D-Bus index
    try:
        dbus_device_idx = int(conceptual_device_index_str) - 1
        if dbus_device_idx < 0:
            raise ValueError("Device index must be 1 or greater.")
    except ValueError as e:
        logger.critical(f"Invalid device index provided: {e}. Exiting.")
        sys.exit(1)

    logger.info(f"Starting D-Bus service process for conceptual device {conceptual_device_index_str} (D-Bus index {dbus_device_idx}).")

    service_config_manager = ServiceConfig()

    # --- Set Log Level from D-Bus Global Setting ---
    LOG_LEVELS = {
        'DEBUG': logging.DEBUG, 'INFO': logging.INFO, 'WARNING': logging.WARNING,
        'ERROR': logging.ERROR, 'CRITICAL': logging.CRITICAL
    }
    log_level_str = service_config_manager.get_setting('LogLevel')
    log_level = LOG_LEVELS.get(log_level_str, logging.INFO) # Default to INFO
    logger.setLevel(log_level)
    logger.debug(f"Log level set to: {logging.getLevelName(logger.level)}")

    # --- Retrieve Global MQTT Settings from D-Bus ---
    mqtt_broker_address = service_config_manager.get_setting('MqttBrokerAddress')
    mqtt_broker_port = service_config_manager.get_setting('MqttBrokerPort')
    mqtt_username = service_config_manager.get_setting('MqttUsername')
    mqtt_password = service_config_manager.get_setting('MqttPassword')

    mqtt_config = {
        'BrokerAddress': mqtt_broker_address if mqtt_broker_address is not None else 'localhost',
        'Port': mqtt_broker_port if mqtt_broker_port is not None else 1883,
        'Username': mqtt_username if mqtt_username is not None else '',
        'Password': mqtt_password if mqtt_password is not None else ''
    }

    # --- Retrieve Device-Specific Settings from D-Bus ---
    # Construct paths using the dbus_device_idx
    device_instance = service_config_manager.get_dynamic_setting(
        service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/Instance")
    )
    custom_name = service_config_manager.get_dynamic_setting(
        service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/CustomName")
    )
    serial_number = service_config_manager.get_dynamic_setting(
        service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/SerialNumber")
    )
    num_switches = service_config_manager.get_dynamic_setting(
        service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/NumberOfSwitches")
    )
    mqtt_on_payload = service_config_manager.get_dynamic_setting(
        service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/MqttOnPayload")
    )
    mqtt_off_payload = service_config_manager.get_dynamic_setting(
        service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/MqttOffPayload")
    )

    # Basic validation for crucial device settings
    if any(val is None for val in [device_instance, custom_name, serial_number, num_switches, mqtt_on_payload, mqtt_off_payload]):
        logger.critical(f"Missing crucial D-Bus settings for device D-Bus index {dbus_device_idx}. Please run config.py. Exiting.")
        sys.exit(1)
        
    device_data = {
        'DeviceIndex': dbus_device_idx, # Store the 0-based D-Bus index
        'DeviceInstance': int(device_instance),
        'CustomName': custom_name,
        'Serial': serial_number,
        'NumberOfSwitches': int(num_switches),
        'MqttOnPayload': mqtt_on_payload,
        'MqttOffPayload': mqtt_off_payload,
    }

    # --- Retrieve Switch-Specific Settings from D-Bus ---
    output_configs = []
    for j in range(int(num_switches)): # Iterate using 0-based index for D-Bus paths
        switch_custom_name = service_config_manager.get_dynamic_setting(
            service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/Switch/{j}/CustomName")
        )
        group = service_config_manager.get_dynamic_setting(
            service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/Switch/{j}/Group")
        )
        mqtt_state_topic = service_config_manager.get_dynamic_setting(
            service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/Switch/{j}/StateTopic")
        )
        mqtt_command_topic = service_config_manager.get_dynamic_setting(
            service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/Switch/{j}/CommandTopic")
        )

        # Basic validation for crucial switch settings
        if any(val is None for val in [switch_custom_name, group, mqtt_state_topic, mqtt_command_topic]):
            logger.warning(f"Missing crucial D-Bus settings for switch {j} under device D-Bus index {dbus_device_idx}. Some functionality may be affected.")
            # Provide sensible defaults if missing, to allow the service to try and run
            switch_custom_name = switch_custom_name if switch_custom_name is not None else f"Switch {j+1}"
            group = group if group is not None else f"Group {dbus_device_idx+1}"
            mqtt_state_topic = mqtt_state_topic if mqtt_state_topic is not None else f"mqtt_switches/device_{device_instance}/switch_{j+1}/state"
            mqtt_command_topic = mqtt_command_topic if mqtt_command_topic is not None else f"mqtt_switches/device_{device_instance}/switch_{j+1}/command"


        output_data = {
            'index': j, # Use 0-based index for internal switch representation
            'name': f'Switch {j+1}', # For display purposes
            'custom_name': switch_custom_name,
            'group': group,
            'MqttStateTopic': mqtt_state_topic,
            'MqttCommandTopic': mqtt_command_topic,
        }
        output_configs.append(output_data)

    service_name = f'com.victronenergy.switch.virtual_{device_data["Serial"]}'

    DbusMyTestSwitch(service_name, device_data, output_configs, mqtt_config)
    
    logger.debug('Connected to D-Bus, and switching over to GLib.MainLoop() (= event based)')
    
    mainloop = GLib.MainLoop()
    mainloop.run()

def main():
    """
    The main launcher function that runs as the parent process.
    """
    from dbus.mainloop.glib import DBusGMainLoop
    DBusGMainLoop(set_as_default=True) # Ensure main loop is set for the launcher process
    
    logger.info("Starting D-Bus Virtual Switch service launcher.")

    service_config_manager = ServiceConfig() # Instantiate ServiceConfig for the launcher as well

    # --- Set Log Level for the Launcher from D-Bus Global Setting ---
    LOG_LEVELS = {
        'DEBUG': logging.DEBUG, 'INFO': logging.INFO, 'WARNING': logging.WARNING,
        'ERROR': logging.ERROR, 'CRITICAL': logging.CRITICAL
    }
    log_level_str = service_config_manager.get_setting('LogLevel')
    log_level = LOG_LEVELS.get(log_level_str, logging.INFO) # Default to INFO
    logger.setLevel(log_level)
    logger.debug(f"Launcher log level set to: {logging.getLevelName(logger.level)}")

    num_devices = service_config_manager.get_setting('NumberOfDevices')
    if num_devices is None:
        logger.warning("Could not retrieve 'NumberOfDevices' from D-Bus. Defaulting to 1 device.")
        num_devices = 1
    else:
        num_devices = int(num_devices) # Ensure it's an integer

    script_path = os.path.abspath(sys.argv[0])
    processes = []
    
    logger.debug(f"Starting {num_devices} virtual switch device processes...")

    # Iterate using 1-based index for launching, as run_device_service expects it
    for i in range(1, num_devices + 1):
        # Before launching, check if the corresponding device's instance path exists in D-Bus
        # This acts as a more robust check than just relying on num_devices
        dbus_device_idx = i - 1 # Convert to 0-based index for D-Bus path checks
        instance_path = service_config_manager.get_dynamic_setting_path(f"Device/{dbus_device_idx}/Instance")
        device_instance_from_dbus = service_config_manager.get_dynamic_setting(instance_path)

        if device_instance_from_dbus is None:
            logger.warning(f"D-Bus settings for device {i} (index {dbus_device_idx}) not found or incomplete. Skipping launch of this device.")
            continue
            
        cmd = [sys.executable, script_path, str(i)] # Pass the 1-based index to the child process
        
        try:
            process = subprocess.Popen(cmd, env=os.environ, close_fds=True)
            processes.append(process)
            logger.debug(f"Started process for virtual device {i} (PID: {process.pid})")
        except Exception as e:
            logger.error(f"Failed to start process for device {i}: {e}")
            
    try:
        while True:
            # Check if any child process has terminated
            for p in processes[:]: # Iterate over a copy of the list
                if p.poll() is not None:
                    logger.warning(f"Child process for PID {p.pid} terminated with exit code {p.poll()}. Removing from list.")
                    processes.remove(p)
            
            if not processes and num_devices > 0: # If all processes died but we expected devices
                logger.error("All virtual device processes have terminated unexpectedly. Exiting launcher.")
                break # Exit the main loop
            
            time.sleep(5) # Check every 5 seconds
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received. Terminating all child processes.")
        for p in processes:
            p.terminate()
        for p in processes:
            p.wait()
    except Exception as e:
        logger.critical(f"An unexpected error occurred in the main launcher loop: {e}")
    finally:
        logger.info("Exiting D-Bus Virtual Switch service launcher.")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        run_device_service(sys.argv[1])
    else:
        main()
