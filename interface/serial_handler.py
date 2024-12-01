import serial
import threading
import time
import queue  # Ensure queue is imported

NUM_SERVOS = 5

class SerialHandler:
    """
    Handles serial communication with Arduino Uno and Nano.
    """
    def __init__(self, config: dict):
        self.config = config
        self.serial_uno = None
        self.serial_nano = None
        self.lock = threading.Lock()
        self.running = False
        self.read_thread = None
        self.data_queue = queue.Queue()  # Original queue for raw lines
        self.parsed_data_queue = queue.Queue()  # New queue for parsed servo data
        self.debug_mode = False  # Initialize debug_mode

    def connect(self):
        """
        Establish serial connections to Uno and Nano.
        """
        with self.lock:
            self.serial_uno = serial.Serial(
                self.config['SERIAL_PORT_UNO'],
                self.config.get('BAUD_RATE_UNO', 115200),
                timeout=1
            )
            self.serial_nano = serial.Serial(
                self.config['SERIAL_PORT_NANO'],
                self.config.get('BAUD_RATE_NANO', 57600),
                timeout=1
            )
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()

    def disconnect(self):
        """
        Close serial connections to Uno and Nano.
        """
        with self.lock:
            self.running = False
            if self.read_thread:
                self.read_thread.join()
            if self.serial_uno and self.serial_uno.is_open:
                self.serial_uno.close()
            if self.serial_nano and self.serial_nano.is_open:
                self.serial_nano.close()

    def is_connected(self):
        """
        Check if both serial connections are open.
        """
        return (self.serial_uno and self.serial_uno.is_open and
                self.serial_nano and self.serial_nano.is_open)

    def send_command(self, command: str, device: str = 'UNO'):
        """
        Send a command to the specified device.
        
        Args:
            command (str): Command to send.
            device (str): 'UNO' or 'NANO'.
        """
        with self.lock:
            if device.upper() == 'UNO' and self.serial_uno and self.serial_uno.is_open:
                self.serial_uno.write((command + '\n').encode('utf-8'))
            elif device.upper() == 'NANO' and self.serial_nano and self.serial_nano.is_open:
                self.serial_nano.write((command + '\n').encode('utf-8'))
            else:
                # Handle error or raise exception
                pass

    def read_serial(self):
        while self.running:
            try:
                if self.serial_uno and self.serial_uno.is_open and self.serial_uno.in_waiting:
                    line = self.serial_uno.readline().decode('utf-8').strip()
                    # Enqueue the raw line instead of the parsed list
                    self.data_queue.put(line)
                # ...existing code...
            except serial.SerialException:
                self.data_queue.put("Serial connection lost.")
                self.disconnect()
            except Exception as e:
                self.data_queue.put(f"Error: {e}")

    def process_incoming_data(self, line: str, append_output):
        """
        Process a single line of incoming serial data.

        Args:
            line (str): The line read from serial.
            append_output (function): Function to append text to the terminal widget.
        """
        if line.startswith('>'):
            # Line contains servo values for plotting
            try:
                # Remove the '>' and split the values
                values_str = line[1:]
                values = list(map(float, values_str.split(',')))
                if len(values) != self.config['NUM_SERVOS']:
                    append_output(f"Invalid number of servo values: {line}", 'UNO')
                    return

                # Put the parsed values into the parsed_data_queue
                self.parsed_data_queue.put(values)
            except ValueError:
                append_output(f"Failed to parse servo values: {line}", 'UNO')
        else:
            # Line is regular terminal output
            append_output(line, 'UNO' if self.debug_mode else 'SYSTEM')

    def reset_servos(self):
        """
        Reset servos to default positions.
        """
        with self.lock:
            for i in range(self.config['NUM_SERVOS']):
                default_angle = self.config['SERVO_START_ANGLES'][i]
                servo_name = self.config['SERVO_NAMES'][i]
                command = f"{servo_name} {default_angle}"
                self.send_command(command, device='UNO')
                time.sleep(0.1)  # Small delay between commands

    def close_serial_ports(self):
        """
        Close serial ports gracefully.
        """
        self.disconnect()
