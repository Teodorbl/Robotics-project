import threading
import serial

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..frontend.gui import GUI as GUIType



class SerialAPI():
    def __init__(self, configs):
        print("-- Serial init --")
        
        self.configs = configs
        self.gui: GUIType = None
        
        # --------------------------------------------
        # Initialize Serial Connections
        # --------------------------------------------

        self.ser_uno = None  # Serial connection to Arduino Uno
        self.ser_nano = None  # Serial connection to Arduino Nano
        self.connection_start_time = None
        
        self.serial_lock = threading.Lock()  # To synchronize access to serial ports
    
    def toggle_connection(self):
        if self.ser_uno and self.ser_uno.is_open and self.ser_nano and self.ser_nano.is_open:
            
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
                
                is_connected = True
                error_msg = None
            
            except serial.SerialException as e:
                self.ser_uno = None
                self.ser_nano = None
                
                is_connected = False
                error_msg = f"{e}"
        
        return is_connected, error_msg
        
    def servo_command(self, servo_index, degree):
        command = f"moveServo {servo_index} {degree}"
        with self.serial_lock:
            try:
                if self.ser_uno and self.ser_uno.is_open:
                    self.ser_uno.write((command + '\n').encode('utf-8'))
            except Exception as e:
                self.gui.append_output(f"Error writing to serial port: {e}")
                
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
                
                self.gui.append_output(line)
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error with Uno: {e}")
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
            
            if data_line and self.gui.knob_mode:
                # Process data line
                
                data = line[1:]
                values = list(map(float, data.split(',')))
                
                if len(values) != self.configs.NUM_SERVOS:
                    raise ValueError(f"Expected {self.configs.NUM_SERVOS} values from NANO knobs, but got {len(values)}")
                
                for servo_index, value in enumerate(values):
                    degree = self.knob_to_degree(value)
                    self.servo_command(servo_index, degree)
                    
            
            if (not data_line) or debug_on:
                # Process non-data line
                
                self.gui.append_output(line)
    
        except serial.SerialException as e:
            self.gui.append_output(f"Serial error with Uno: {e}")
            self.toggle_connection()
    
        except Exception as e:
            self.gui.append_outout(f"Error during read of UNO serial: {e}")
                
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
        
        if self.ser_uno and self.ser_uno.is_open:
            with self.serial_lock:
                self.ser_uno.close()
                
        if self.ser_nano and self.ser_nano.is_open:
            with self.serial_lock:
                self.ser_nano.close()
        

