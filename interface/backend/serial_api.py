import threading
import serial
import time
import os
import json
import csv  # For CSV operations
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
        
        self.serial_lock = threading.Lock()  # To synchronize access to serial ports
        
        # Initialize servo positions
        self.servo_pos_init = configs.SERVO_DEFAULT_ANGLES
        self.servo_positions = self.servo_pos_init.copy()
        
        # Initialize Logging
        self.log_file = "command_feedback_log.csv"
        self.log_lock = threading.Lock()
        
        if not os.path.exists(self.log_file):
            with open(self.log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["timestamp", "event_type", "content"])
        
        print("-- Serial init end --")

    def log_event(self, event_type, content):
        """
        Log an event to the CSV file.
        
        :param event_type: Type of event ('command_sent', 'feedback_received')
        :param content: The content of the event (command or feedback)
        """
        timestamp = time.time()
        with self.log_lock:
            with open(self.log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, event_type, content])
        
    def toggle_connection(self):
        if self.ser_uno and self.ser_uno.is_open and self.ser_nano and self.ser_nano.is_open:
            self.reset_servos()
            
            # Disconnect both
            with self.serial_lock:
                self.ser_uno.close()
                self.ser_nano.close()
                self.ser_uno = None
                self.ser_nano = None
            
            is_connected = False
            error_msg = None

        else:
            # Connect both
            try:
                with self.serial_lock:
                    self.ser_uno = serial.Serial(self.configs.SERIAL_PORT_UNO, self.configs.BAUD_RATE_UNO, timeout=1)
                    self.ser_nano = serial.Serial(self.configs.SERIAL_PORT_NANO, self.configs.BAUD_RATE_NANO, timeout=1)
                
                if self.gui:
                    self.gui.reset_iterations()
                
                self.servo_positions = self.servo_pos_init.copy()
                
                is_connected = True
                error_msg = None
            
            except serial.SerialException as e:
                #self.reset_servos()
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
        
        # Log the command with timestamp
        self.log_event("command_sent", command)
        
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
                
    def read_uno(self):
        if not (self.ser_uno and self.ser_uno.is_open and self.ser_uno.in_waiting > 0):
            return
        
        try:
            while self.ser_uno.in_waiting > 0:
                line = self.ser_uno.readline().decode('utf-8').strip()
            
                # Check for data line
                feedback_data_line = line.startswith('FB:')
                debug_on = self.gui.debug_mode
                
                if feedback_data_line:
                    # Process data line
                    data = line[3:]
                    values = list(map(float, data.split(',')))
                    
                    if len(values) != self.configs.NUM_SERVOS-1:
                        raise ValueError(f"Expected {self.configs.NUM_SERVOS-1} values from UNO serial line, but got {len(values)}")
                    
                    self.gui.plot_servo_pos(values)
                    
                    # Log the feedback
                    self.log_event("feedback_received", f"Uno_FB: {data}")
                
                if (not feedback_data_line) or debug_on:
                    # Process non-data line
                    self.gui.append_output(line, 'UNO')
                    
                    if not feedback_data_line:
                        # Log the non-data feedback
                        self.log_event("feedback_received", f"Uno_Non_FB: {line}")
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error with UNO: {e}", 'serial_api')
            self.toggle_connection()
    
        except Exception as e:
            self.gui.append_output("Error during read of UNO serial", 'serial_api')
            self.gui.append_output(f"Line: {line}", 'serial_api')
            self.gui.append_output(f"Error: {e}", 'serial_api')
            self.log_event("error", f"Uno_Error: {e}")
    
    def read_nano(self):
        if not (self.ser_nano and self.ser_nano.is_open and self.ser_nano.in_waiting > 0):
            return
        
        try:
            while self.ser_nano.in_waiting > 0:
                line = self.ser_nano.readline().decode('utf-8').strip()
                
                current_data_line = line.startswith('A:')       # Current readings
                servo5_data_line = line.startswith('FB5:')      # Servo 5 feedback
                debug_on = self.gui.debug_mode
                
                if current_data_line:
                    data = line[2:]
                    values = list(map(float, data.split(',')))
                    num_values = len(values)
                    
                    if num_values != self.configs.NUM_SERVOS:
                        raise ValueError(f"Expected {self.configs.NUM_SERVOS} values from NANO current sensors, but got {len(values)}")
                    
                    self.gui.plot_servo_currents(values)
                    
                    # Log the feedback
                    self.log_event("feedback_received", f"Nano_Current: {data}")
                
                elif servo5_data_line:
                    value = int(line[4:])
                    
                    self.gui.plot_servo5_pos(value)
                    
                    # Log the feedback
                    self.log_event("feedback_received", f"Nano_Servo5_FB: {value}")
                
                elif debug_on:
                    self.gui.append_output(line, 'NANO')
                    
                    if not current_data_line and not servo5_data_line:
                        # Log the non-data feedback
                        self.log_event("feedback_received", f"Nano_Non_FB: {line}")
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error NANO: {e}", 'serial_api')
            self.toggle_connection()
        
        except Exception as e:
            self.gui.append_output("Error during read of NANO serial", 'serial_api')
            self.gui.append_output(f"Line: {line}", 'serial_api')
            self.gui.append_output(f"Error: {e}", 'serial_api')
            self.log_event("error", f"Nano_Error: {e}")
                
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