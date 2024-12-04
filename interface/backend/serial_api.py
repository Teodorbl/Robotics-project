import threading
import serial
import time
import os
import json

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..frontend.gui import GUI as GUIType



class SerialAPI():
    def __init__(self, configs):
        print("-- Serial init start --")
        
        self.configs = configs
        self.gui: GUIType = None
        
        # Initialize Serial Connections
        self.ser_uno = None  # Serial connection to Arduino Uno
        self.ser_nano = None  # Serial connection to Arduino Nano
        self.connection_start_time = None
        
        self.serial_lock = threading.Lock()  # To synchronize access to serial ports
        
        # Initialize servo positions
        self.servo_pos_init = configs.SERVO_DEFAULT_ANGLES
        self.servo_positions = self.servo_pos_init.copy()
        
        # Init error and debug parsing
        self.json_filepath = configs.JSON_FILEPATH
        self.errors = {}
        self.debugs = {}

        if not os.path.exists(self.json_filepath):
            raise FileNotFoundError(f"JSON file not found at: {self.json_filepath}")

        with open(self.json_filepath, 'r') as file:
            data = json.load(file)
            self.errors = {int(k): v for k, v in data.get("errors", {}).items()}
            self.debugs = {int(k): v for k, v in data.get("debugs", {}).items()}
        
        print("-- Serial init end --")
        
    def toggle_connection(self):
        if self.ser_uno and self.ser_uno.is_open and self.ser_nano and self.ser_nano.is_open:
            self.reset_servos()
            
            # Disconnect both
            with self.serial_lock:
                self.ser_uno.close()
                self.ser_nano.close()
                self.ser_uno = None
                self.ser_nano = None
            
            self.connection_start_time = None
            
            is_connected = False
            error_msg = None

        else:
            # Connect both
            try:
                with self.serial_lock:
                    self.ser_uno = serial.Serial(self.configs.SERIAL_PORT_UNO, self.configs.BAUD_RATE_UNO, timeout=1)
                    self.ser_nano = serial.Serial(self.configs.SERIAL_PORT_NANO, self.configs.BAUD_RATE_NANO, timeout=1)
                
                self.connection_start_time = time.time()
                self.servo_positions = self.servo_pos_init.copy()
                
                is_connected = True
                error_msg = None
            
            except serial.SerialException as e:
                self.reset_servos()
                self.ser_uno = None
                self.ser_nano = None
                
                is_connected = False
                error_msg = f"{e}"
        
        if not is_connected:
            self.connection_start_time = None
            
        return is_connected, error_msg
        
    def servo_command(self, servo_index, degree):
        if isinstance(servo_index, list):
            for idx, deg in zip(servo_index, degree):
                self.servo_positions[idx] = int(deg)
        
        else:
            self.servo_positions[servo_index] = int(degree)
        
        
        pos = self.servo_positions
        
        command = f"<{pos[0]:03d},{pos[1]:03d},{pos[2]:03d},{pos[3]:03d},{pos[4]:03d}>"
        
        self.gui.append_output(f"Sending command: {command}", 'serial_api')
        
        with self.serial_lock:
            try:
                if self.ser_uno and self.ser_uno.is_open:
                    self.ser_uno.write((command + '\n').encode('utf-8'))
            except Exception as e:
                self.gui.append_output(f"Error writing to serial port: {e}", 'serial_api')
    
    def reset_servos(self):
        default_pos = self.servo_pos_init.copy()
        servo_indices = list(range(self.configs.NUM_SERVOS))
        
        self.servo_command(servo_indices, default_pos)

    def get_error_message(self, code):
        error = self.errors.get(code)
        if error:
            message = error["message"]
            data = error["data"]
            return f"Error {code}: {message}" + (f" | Data: {data}" if data else "")
        else:
            raise ValueError(f"Unknown Error Code: {code}")

    def get_debug_message(self, code):
        debug = self.debugs.get(code)
        if debug:
            message = debug["message"]
            data = debug["data"]
            return f"Debug {code}: {message}" + (f" | Data: {data}" if data else "")
        else:
            raise ValueError(f"Unknown Debug Code: {code}")
                
    def read_uno(self):
        if not (self.ser_uno and self.ser_uno.is_open and self.ser_uno.in_waiting > 0):
            return
        
        try:
            line = self.ser_uno.readline().decode('utf-8').strip()
            
            # # Check for error line
            # if line.startswith("E:"):
            #     parts = line.strip().split(":", 2)
            #     code = int(parts[1])
            #     data = parts[2] if len(parts) > 2 else None
            #     message = self.get_error_message(code)
            #     output = message + (f" | Data: {data}" if data else "")
            #     print(output)
            #     if self.gui.debug_mode: self.gui.append_output(output, 'UNO')
            
            # # Check for debug line
            # elif line.startswith("D:"):
            #     parts = line.strip().split(":", 2)
            #     code = int(parts[1])
            #     data = parts[2] if len(parts) > 2 else None
            #     message = self.get_debug_message(code)
            #     output = message + (f" | Data: {data}" if data else "")
            #     print(output)
            #     if self.gui.debug_mode: self.gui.append_output(output, 'UNO')
        
            # Check for data line
            feedback_data_line = line.startswith('FB:')
            debug_on = self.gui.debug_mode
            
            if feedback_data_line:
                # Process data line
                
                data = line[3:]
                values = list(map(float, data.split(',')))
                
                if len(values) != self.configs.NUM_SERVOS:
                    raise ValueError(f"Expected {self.configs.NUM_SERVOS} values from UNO serial line, but got {len(values)}")
                
                self.gui.plot_servo_pos(values)
            
            if (not feedback_data_line) or debug_on:
                # Process non-data line
                
                self.gui.append_output(line, 'UNO')
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error with UNO: {e}")
            self.toggle_connection()
    
        except Exception as e:
            self.gui.append_output("Error during read of UNO serial", 'serial_api')
            self.gui.append_output(f"Line: {line}", 'serial_api')
            self.gui.append_output(f"Error: {e}", 'serial_api')

    def read_nano(self):
        if not (self.ser_nano and self.ser_nano.is_open and self.ser_nano.in_waiting > 0):
            return
        
        try:
            while self.ser_nano.in_waiting > 0:
                line = self.ser_nano.readline().decode('utf-8').strip()
                
                current_data_line = line.startswith('A:')       # Current readings
                servo5_data_line = line.startswith('FB5:')       # Current readings
                debug_on = self.gui.debug_mode
                
                if current_data_line:
                    data = line[2:]
                    values = list(map(float, data.split(',')))
                    num_values = len(values)
                    
                    if num_values != self.configs.NUM_SERVOS:
                        raise ValueError(f"Expected {self.configs.NUM_SERVOS} values from NANO current sensors, but got {len(values)}")
                    
                    self.gui.plot_servo_currents(values)
                    
                elif servo5_data_line:
                    value = int(line[4:])
                    
                    self.gui.plot_servo5_pos(value)
                    
                
                # if data_line and self.gui.knob_mode:
                #     # Process data line
                    
                #     data = line[1:]
                #     values = list(map(float, data.split(',')))
                #     num_values = len(values)
                    
                #     if num_values != self.configs.NUM_SERVOS:
                #         raise ValueError(f"Expected {self.configs.NUM_SERVOS} values from NANO knobs, but got {len(values)}")
                    
                #     indices = list(range(num_values))
                #     degrees = [self.knob_to_degree(val, idx) for val, idx in zip(values, indices)]
                #     self.servo_command(indices, degrees)
                
                elif debug_on:
                    self.gui.append_output(line, 'NANO')
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error NANO: {e}", 'serial_api')
            self.toggle_connection()
    
        # except Exception as e:
        #     self.gui.append_output("Error during read of NANO serial", 'serial_api')
        #     self.gui.append_output(f"Line: {line}", 'serial_api')
        #     self.gui.append_output(f"Error: {e}", 'serial_api')
                
    def knob_to_degree(self, raw_knob, servo_index):
        """
        Convert raw analog value to degree based on a fixed mapping per servo.
        """
        # Define calibration parameters (unchanged)
        knob_min = 32
        knob_max = 1023
        
        degree_min = self.configs.SERVO_MIN_ANGLES[servo_index]
        degree_max = self.configs.SERVO_MAX_ANGLES[servo_index]
        
        self.configs.SERVO_MAX_ANGLES[servo_index]

        # Clamp the raw value to the expected range
        raw_knob = max(knob_min, min(raw_knob, knob_max))

        # Linear mapping from raw value to degree
        degree = ((knob_max - raw_knob) / (knob_max - knob_min)) * (degree_max - degree_min) + degree_min
        return int(degree)

    def set_use_knobs(self, enable: bool, retries: int = 3, timeout: float = 1.0):
        """
        Request the Arduino to change useKnobsEnabled state.
        
        :param enable: True to enable, False to disable useKnobsEnabled
        :param retries: Number of retries if ACK is not received
        :param timeout: Time to wait for ACK in seconds
        """
        command = "useKnobs {}\n".format(int(enable))
        expected_ack = f"ACK: useKnobsEnabled={'true' if enable else 'false'}"
        
        for attempt in range(retries):
            with self.serial_lock:
                try:
                    self.ser_uno.write(command.encode('utf-8'))
                except Exception as e:
                    self.gui.append_output(f"Error writing to UNO serial: {e}", 'serial_api')
                    return False
            
            # Wait for ACK
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.ser_uno.in_waiting > 0:
                    try:
                        line = self.ser_uno.readline().decode('utf-8').strip()
                        if line == expected_ack:
                            return True
                    except Exception as e:
                        self.gui.append_output(f"Error reading UNO serial: {e}", 'serial_api')
                        break
                time.sleep(0.1)
            
            self.gui.append_output(f"ACK not received for set_use_knobs attempt {attempt + 1}", 'serial_api')
        
        self.gui.append_output("Failed to receive ACK after retries.", 'serial_api')
        return False

    def close(self):
        # Disconnect both
        self.gui.append_output("Closing both serial ports", 'serial_api')
        self.reset_servos()
        
        if self.ser_uno and self.ser_uno.is_open:
            try:
                with self.serial_lock:
                    self.ser_uno.close()
            except Exception as e:
                print("Could not close UNO serial. Error: {e}")
                
        if self.ser_nano and self.ser_nano.is_open:
            try:
                with self.serial_lock:
                    self.ser_nano.close()
            except Exception as e:
                print("Could not close NANO serial. Error: {e}")