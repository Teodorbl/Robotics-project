import threading
import serial
import time

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..frontend.gui import GUI as GUIType



class SerialAPI():
    def __init__(self, configs):
        print("-- Serial init --")
        
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
        
        command = f"moveServos {' '.join(map(str, self.servo_positions))}"
        print(command)
        
        with self.serial_lock:
            try:
                if self.ser_uno and self.ser_uno.is_open:
                    self.ser_uno.write((command + '\n').encode('utf-8'))
            except Exception as e:
                self.gui.append_output(f"Error writing to serial port: {e}")
    
    def reset_servos(self):
        default_pos = self.servo_pos_init.copy()
        servo_indices = list(range(self.configs.NUM_SERVOS))
        
        self.servo_command(servo_indices, default_pos)
                
    def read_uno(self):
        if not (self.ser_uno and self.ser_uno.is_open and self.ser_uno.in_waiting > 0):
            return
        
        try:
            line = self.ser_uno.readline().decode('utf-8').strip()
            
            data_line = line.startswith('>')
            debug_on = self.gui.debug_mode
            
            if data_line:
                # Process data line
                
                data = line[1:]
                values = list(map(float, data.split(',')))
                
                if len(values) != self.configs.NUM_SERVOS:
                    raise ValueError(f"Expected {self.configs.NUM_SERVOS} values from UNO serial line, but got {len(values)}")
                
                self.gui.plot_servo_pos(values)
            
            if (not data_line) or debug_on:
                # Process non-data line
                
                self.gui.append_output(line, 'UNO')
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error with UNO: {e}")
            self.toggle_connection()
    
        except Exception as e:
            self.gui.append_outout(f"Error during read of UNO serial: {e}")

    def read_nano(self):
        if not (self.ser_nano and self.ser_nano.is_open and self.ser_nano.in_waiting > 0):
            return
        
        try:
            line = self.ser_nano.readline().decode('utf-8').strip()
            
            data_line = line.startswith('>')
            debug_on = self.gui.debug_mode
            
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
                    
            
            if (not data_line) or debug_on:
                # Process non-data line
                
                self.gui.append_output(line, 'NANO')
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error with NANO: {e}")
            self.toggle_connection()
    
        except Exception as e:
            self.gui.append_output(f"Error during read of NANO serial: {e}")
                
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
    
    def close(self):
        # Disconnect both
        self.gui.append_output("Closing both serial ports")
        self.reset_servos()
        
        if self.ser_uno and self.ser_uno.is_open:
            with self.serial_lock:
                self.ser_uno.close()
                
        if self.ser_nano and self.ser_nano.is_open:
            with self.serial_lock:
                self.ser_nano.close()

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
                    self.gui.append_output(f"Error writing to UNO serial: {e}")
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
                        self.gui.append_output(f"Error reading UNO serial: {e}")
                        break
                time.sleep(0.1)
            
            self.gui.append_output(f"ACK not received for set_use_knobs attempt {attempt + 1}")
        
        self.gui.append_output("Failed to receive ACK after retries.")
        return False


