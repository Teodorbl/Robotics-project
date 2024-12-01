import serial
import threading
import time
import queue  # Added import

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
        self.data_queue = queue.Queue()  # Added queue for servo data

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
        """
        Read incoming data from serial ports.
        """
        while self.running:
            with self.lock:
                if self.serial_uno and self.serial_uno.is_open and self.serial_uno.in_waiting > 0:
                    try:
                        line = self.serial_uno.readline().decode('utf-8').strip()
                        if line:
                            self.process_incoming_data(line, self.append_output)
                    except Exception as e:
                        self.append_output(f"Error reading UNO: {e}", 'UNO')

                if self.serial_nano and self.serial_nano.is_open and self.serial_nano.in_waiting > 0:
                    try:
                        line = self.serial_nano.readline().decode('utf-8').strip()
                        if line:
                            self.process_incoming_data(line, self.append_output)
                    except Exception as e:
                        self.append_output(f"Error reading Nano: {e}", 'NANO')
            time.sleep(0.05)  # Maintain 50 ms sleep

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

                # Put the parsed values into the queue
                self.data_queue.put(values)
            except ValueError:
                append_output(f"Failed to parse servo values: {line}", 'UNO')
        else:
            # Line is regular terminal output
            append_output(line, 'UNO' if self.debug_mode else 'SYSTEM')

    def process_incoming_data(self, append_output, update_plots, plots, curves, labels, data_buffers, debug_mode, control_mode_follow, servo_sliders):
        """
        Process data read from serial ports and update the GUI accordingly.
        """
        with self.lock:
            # Process UNO data
            if hasattr(self, 'latest_uno_data') and self.latest_uno_data:
                data = self.latest_uno_data
                append_output(f"UNO: {data}", 'UNO')
                # Example expected format: "servo_index:angle"
                try:
                    parts = data.split(':')
                    if len(parts) == 2:
                        servo_index = int(parts[0])
                        angle = int(parts[1])
                        if 0 <= servo_index < len(labels):
                            labels[servo_index].setText(f"{labels[servo_index].text().split(':')[0]}: {angle}°")
                            if update_plots:
                                current_time = time.time()
                                data_buffers[servo_index].append((current_time, angle))
                                # Update plot data
                                times, angles = zip(*data_buffers[servo_index])
                                curves[servo_index].setData(times, angles)
                except ValueError:
                    append_output(f"Invalid UNO data format: {data}", 'UNO')
                self.latest_uno_data = None  # Reset after processing

            # Process NANO data
            if hasattr(self, 'latest_nano_data') and self.latest_nano_data:
                data = self.latest_nano_data
                append_output(f"NANO: {data}", 'NANO')
                # Example expected format: "servo5_feedback:angle"
                try:
                    parts = data.split(':')
                    if len(parts) == 2 and parts[0] == 'servo5_feedback':
                        servo5_angle = int(parts[1])
                        labels[4].setText(f"{labels[4].text().split(':')[0]}: {servo5_angle}°")
                        if update_plots:
                            current_time = time.time()
                            data_buffers[4].append((current_time, servo5_angle))
                            curves[4].setData(*zip(*data_buffers[4]))
                except ValueError:
                    append_output(f"Invalid NANO data format: {data}", 'NANO')
                self.latest_nano_data = None  # Reset after processing

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
