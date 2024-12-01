import threading
import serial
import queue  # Import queue for thread-safe communication
import time

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..frontend.gui import GUI as GUIType

class SerialAPI():
    def __init__(self, configs):
        print("-- SerialAPI init --")
        self.configs = configs
        self.gui: GUIType = None
        
        # --------------------------------------------
        # Initialize Serial Connections
        # --------------------------------------------

        self.ser_uno = None  # Serial connection to Arduino Uno
        self.ser_nano = None  # Serial connection to Arduino Nano
        self.connection_start_time = None
        
        self.serial_lock = threading.Lock()  # To synchronize access to serial ports

        # Queues for communicating with the GUI
        self.uno_queue = queue.Queue()
        self.nano_queue = queue.Queue()

        # Thread control flags
        self.uno_thread_running = False
        self.nano_thread_running = False
        self.read_interval = 0.01
        
        
        print("SerialAPI initialized.")
    
    def toggle_connection(self):
        print("Toggling connection...")
        if self.ser_uno and self.ser_uno.is_open and self.ser_nano and self.ser_nano.is_open:
            print("Disconnecting serial connections...")
            # Disconnect both
            with self.serial_lock:
                self.ser_uno.close()
                self.ser_nano.close()
                self.ser_uno = None
                self.ser_nano = None
            print("Serial connections disconnected.")
            # Stop the threads
            self.uno_thread_running = False
            self.nano_thread_running = False
            
            is_connected = False
            error_msg = None

        else:
            print("Connecting to serial ports...")
            # Connect both
            try:
                with self.serial_lock:
                    print(f"Opening UNO serial port {self.configs.SERIAL_PORT_UNO}...")
                    self.ser_uno = serial.Serial(self.configs.SERIAL_PORT_UNO, self.configs.BAUD_RATE_UNO, timeout=1)
                    print("UNO serial port opened.")
                    print(f"Opening NANO serial port {self.configs.SERIAL_PORT_NANO}...")
                    self.ser_nano = serial.Serial(self.configs.SERIAL_PORT_NANO, self.configs.BAUD_RATE_NANO, timeout=1)
                    print("NANO serial port opened.")

                # Start the threads
                print("Starting serial read threads...")
                self.uno_thread_running = True
                self.nano_thread_running = True
                self.uno_thread = threading.Thread(target=self.read_uno_thread)
                self.nano_thread = threading.Thread(target=self.read_nano_thread)
                self.uno_thread.start()
                self.nano_thread.start()
                print("Serial read threads started.")
                # Record the connection start time
                self.connection_start_time = time.time()
                
                is_connected = True
                error_msg = None
            
            except serial.SerialException as e:
                with self.serial_lock:
                    self.ser_uno = None
                    self.ser_nano = None
                print(f"Serial connection failed: {e}")
                is_connected = False
                error_msg = f"{e}"
        
        print("toggle_connection completed.")
        return is_connected, error_msg
        
    def servo_command(self, servo_index, degree):
        command = f"moveServo {servo_index} {degree}"
        with self.serial_lock:
            try:
                if self.ser_uno and self.ser_uno.is_open:
                    self.ser_uno.write((command + '\n').encode('utf-8'))
            except Exception as e:
                self.gui.append_output(f"Error writing to serial port: {e}")
                
    def read_uno_thread(self):
        print("UNO thread started.")
        while self.uno_thread_running:
            try:
                if self.ser_uno and self.ser_uno.is_open:
                    print("Reading from UNO...")
                    line = self.ser_uno.readline().decode('utf-8').strip()
                    if line:
                        print(f"UNO received: {line}")
                        # Parse the line here
                        self.parse_uno_line(line)
                time.sleep(self.read_interval)  # Sleep briefly to prevent a tight loop
            except serial.SerialException as e:
                print(f"Exception in UNO thread: {e}")
                self.gui.append_output(f"Serial error with Uno: {e}")
                self.uno_thread_running = False
                self.toggle_connection()
            except Exception as e:
                print(f"Exception in UNO thread: {e}")
                self.gui.append_output(f"Error during read of UNO serial: {e}")
        print("UNO thread exiting.")

    def read_nano_thread(self):
        print("NANO thread started.")
        while self.nano_thread_running:
            try:
                if self.ser_nano and self.ser_nano.is_open:
                    print("Reading from NANO...")
                    line = self.ser_nano.readline().decode('utf-8').strip()
                    if line:
                        print(f"NANO received: {line}")
                        # Parse the line here
                        self.parse_nano_line(line)
                time.sleep(self.read_interval)  # Sleep briefly to prevent a tight loop
            except serial.SerialException as e:
                print(f"Exception in NANO thread: {e}")
                self.gui.append_output(f"Serial error with Nano: {e}")
                self.nano_thread_running = False
                self.toggle_connection()
            except Exception as e:
                print(f"Exception in NANO thread: {e}")
                self.gui.append_output(f"Error during read of NANO serial: {e}")
        print("NANO thread exiting.")

    def parse_uno_line(self, line):
        data_line = line.startswith('>')
        debug_on = self.gui.debug_mode

        if data_line:
            data = line[1:]
            values = list(map(float, data.split(',')))
            if len(values) != self.configs.NUM_SERVOS:
                self.gui.append_output(
                    f"Expected {self.configs.NUM_SERVOS} values from UNO serial line, but got {len(values)}")
                return
            # Put the values into the queue
            self.uno_queue.put(values)
        elif debug_on:
            self.gui.append_output(line)

    def parse_nano_line(self, line):
        data_line = line.startswith('>')
        debug_on = self.gui.debug_mode

        if data_line and self.gui.knob_mode:
            data = line[1:]
            values = list(map(float, data.split(',')))
            if len(values) != self.configs.NUM_SERVOS:
                self.gui.append_output(
                    f"Expected {self.configs.NUM_SERVOS} values from NANO knobs, but got {len(values)}")
                return
            # Put the values into the queue
            self.nano_queue.put(values)
        elif debug_on:
            self.gui.append_output(line)

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
        # DO NOT EDIT
        degree = ((knob_max - raw_knob) / (knob_max - knob_min)) * (degree_max - degree_min) + degree_min
        # PLEASE
        
        return int(degree)
    
    def close(self):
        # Reset servos
        default_degree_list = self.configs.SERVO_START_ANGLES
        with self.serial_lock:
            for servo_index, degree in enumerate(default_degree_list):
                command = f"moveServo {servo_index} {degree}"
                try:
                    if self.ser_uno and self.ser_uno.is_open:
                        self.ser_uno.write((command + '\n').encode('utf-8'))
                except Exception as e:
                    self.gui.append_output(f"Error writing to serial port: {e}")
                
        # Disconnect both
        self.uno_thread_running = False
        self.nano_thread_running = False
        
        if self.ser_uno and self.ser_uno.is_open:
            with self.serial_lock:
                self.ser_uno.close()
                
        if self.ser_nano and self.ser_nano.is_open:
            with self.serial_lock:
                self.ser_nano.close()


