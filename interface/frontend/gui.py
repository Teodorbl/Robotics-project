# Own modules
import time
from pyqtgraph.Qt.QtCore import QTimer

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..backend.serial_api import SerialAPI as SerialAPIType

from frontend.window import Window

class GUI():
    def __init__(self, configs, serial_api):
        print("-- GUI init --")
        
        polling_interval = 100     # ms
        
        self.configs = configs
        self.num_servos = self.configs.NUM_SERVOS
        self.servo_names = self.configs.SERVO_NAMES
        self.servo_indices = {value: idx for idx, value in enumerate(self.servo_names)}
        self.serial_api: 'SerialAPIType' = serial_api
        
        # Initialize buffer for each servo
        self.servo_pos_buffers = [[] for _ in range(configs.NUM_SERVOS)]
        
        # Initialize debug mode flag
        self.debug_mode = False
        
        # Initialize Control Mode Flag
        self.knob_mode = False
                
        # Init window
        self.window = Window(self, self.configs)
        
        # Associate GUI with serial_api
        self.serial_api.gui = self
        
        # Init GUI event poller
        self.timer = QTimer()
        self.timer.timeout.connect(self.GUI_poll)
        self.timer.start(polling_interval)
        
        print("GUI initialized.")
        
    def GUI_poll(self):
        # Process UNO data
        while not self.serial_api.uno_queue.empty():
            print("Processing UNO data from queue.")
            values = self.serial_api.uno_queue.get()
            self.plot_servo_pos(values)
        # Process NANO data
        while not self.serial_api.nano_queue.empty():
            print("Processing NANO data from queue.")
            values = self.serial_api.nano_queue.get()
            if self.knob_mode:
                for servo_index, raw_knob_value in enumerate(values):
                    degree = self.serial_api.knob_to_degree(raw_knob_value, servo_index)
                    self.serial_api.servo_command(servo_index, degree)
        # Optionally, process any messages or errors
        print("GUI_poll completed.")

    def append_output(self, text, device=None):
        print(f"{device}: {text}")
        
        if device == 'UNO':
            self.window.uno_output_display.append(text)
        elif device == 'NANO':
            self.window.nano_output_display.append(text)
        else:
            self.window.uno_output_display.append(text)
            self.window.nano_output_display.append(text)
    
    def plot_servo_pos(self, values):
        current_time = time.time() - self.serial_api.connection_start_time
            
        for i in range(self.configs.NUM_SERVOS):
            voltage = values[i]
            self.servo_pos_buffers[i].append((current_time, voltage))
            
            # Corrected line: Use self.servo_pos_buffers instead of self.gui.servo_pos_buffers
            times, voltages = zip(*self.servo_pos_buffers[i][-100:])
            self.window.curves[i].setData(times, voltages)
            
            # Update current value label
            servo_name = self.servo_names[i]
            self.window.labels[i].setText(f"{servo_name}: {voltage:.2f} V")
    
    def toggle_connection(self):
        is_connected, error_msg = self.serial_api.toggle_connection()
        
        if is_connected:
            self.append_output(f"Connected to Uno at {self.configs.SERIAL_PORT_UNO} and Nano at {self.configs.SERIAL_PORT_NANO}.")
            self.window.connect_button.setText("Disconnect")
            
            # Clear data buffers upon connection
            for buffer in self.servo_pos_buffers:
                buffer.clear()
            
            # Reset all current value labels
            for label in self.window.labels:
                label.setText("N/A")
            
        else:
            if error_msg is not None:
                self.append_output(f"Failed to connect: {error_msg}")
            else:
                self.append_output("Disconnected from serial monitors")
            
            self.window.connect_button.setText("Connect")
    
    def uno_text_command(self):
        
        # Extract text input
        user_input = self.window.uno_command_input.text().strip()
        if not user_input:
            return

        # Should be "[servoName OR servo_index] [degree]"
        parts = user_input.split()
        if len(parts) != 2:
            self.append_output('Should be "[servoName OR servo_index] [degree]"')
            return
        
        servo, degree = parts
        
        if not type(degree) is int:
            self.append_output("Degree must be integer")
            return
        
        if (not servo in self.servo_names) and (not servo in range(self.num_servos)):
            self.append_output("Servo doesn't exist")
            return
        
        # Get index from name
        if type(servo) is str:
            servo = self.servo_indices[servo]

        self.serial_api.servo_command(servo, degree)

    def toggle_debug_mode(self, state: bool):
        self.debug_mode = state
        if self.debug_mode:
            self.append_output("Debug Mode Enabled.")
        else:
            self.append_output("Debug Mode Disabled.")
    
    def toggle_control_mode(self, state):
        self.knob_mode = state
        if self.knob_mode:
            self.append_output("Control Mode: Potentiometers (Automatic)")
        else:
            self.append_output("Control Mode: Manual Sliders")
    
    def slider_changed(self, servo_index, degree):
        # Function to handle slider changes with inversion
        if self.knob_mode:
            # In Follow Mode, do not send commands from sliders
            return
        
        try:
            # Send command to servo through backend
            self.serial_api.servo_command(servo_index, degree)
            servo_name = self.servo_names[servo_index]
            self.append_output(f"Command: {servo_name} to {degree}°", 'UNO')
            
        except Exception as e:
            self.append_output(f"Error sending command to {servo_name}: {e}", 'UNO')
            
    def reset_sliders(self):
        if self.knob_mode:
            self.append_output("Must be in slider mode to reset sliders")
            return
        
        for i in range(self.num_servos):
            servo_sliders = self.window.servo_sliders
            
            default_angle = self.configs.SERVO_START_ANGLES[i]
            
            servo_sliders[i].blockSignals(True)  # Prevent triggering slider_changed
            servo_sliders[i].setValue(default_angle)
            servo_sliders[i].blockSignals(False)
            
            # Update the corresponding label
            # Retrieve the label from the layout
            slider_layout = self.window.servo_control_layout.itemAt(i).layout()
            slider_label = slider_layout.itemAt(2).widget()
            slider_label.setText(f"{default_angle}°")
            
            try:
                # Send command to servo through backend
                self.serial_api.servo_command(i, default_angle)
                servo_name = self.servo_names[i]
                self.append_output(f"Command: {servo_name} to {default_angle}°", 'UNO')
                
            except Exception as e:
                self.append_output(f"Error sending command to {servo_name}: {e}", 'UNO')
    
    def close(self):
        # Close the main window
        self.window.main_window.close()












