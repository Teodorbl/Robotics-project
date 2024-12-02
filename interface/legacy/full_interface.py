# FILE:FULL_INTERFACE.PY

import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import signal
import threading
import time
import interface.legacy.angle_calibration as angle_calibration  # Import the calibration module
import re
import atexit

# --------------------------------------------
# Configuration Section
# --------------------------------------------

# Configure the serial ports for both Nano and Uno
SERIAL_PORT_UNO = '/dev/cu.usbmodem11101'      # Replace with your Arduino Uno's serial port
SERIAL_PORT_NANO = '/dev/cu.usbserial-A105A9FZ'  # Replace with your Arduino Nano's serial port

# Function to parse ServoConfig.h and extract constants
def parse_servo_config(config_path):
    constants = {}
    with open(config_path, 'r') as file:
        for line in file:
            # Match lines like #define CONSTANT_NAME value with optional leading whitespace
            define_match = re.match(r'\s*#define\s+(\w+)\s+(\d+)', line)
            if define_match:
                key, value = define_match.groups()
                constants[key] = int(value)
            # Match lines like const type NAME[NUM] = {values}; with optional leading whitespace and pointer types
            array_match = re.match(r'\s*const\s+([\w\*]+)\s+(\w+)\s*\[\s*\w+\s*\]\s*=\s*\{([^}]+)\};', line)
            if array_match:
                type_, key, values = array_match.groups()
                if type_.lower() == 'bool':
                    constants[key] = [v.strip().lower() == 'true' for v in values.split(',')]
                elif type_.lower() == 'char*':
                    constants[key] = [v.strip().strip('"') for v in values.split(',')]
                else:
                    constants[key] = [int(v.strip()) for v in values.split(',')]
            
            # ...existing code...
            
            # Add handling for scalar const declarations
            scalar_match = re.match(r'\s*const\s+(\w+)\s+(\w+)\s*=\s*([^\s;]+)\s*;', line)
            if scalar_match:
                type_, key, value = scalar_match.groups()
                if type_.lower() in ['int', 'float']:
                    constants[key] = int(value) if type_.lower() == 'int' else float(value)
                elif type_.lower() == 'bool':
                    constants[key] = value.strip().lower() == 'true'
                elif type_.lower() in ['char*', 'const char*']:
                    constants[key] = value.strip().strip('"')
    return constants

# Parse ServoConfig.h
servo_config_path = '/Users/teodorlindell/Repos/RoboticArmFreeRTOS/include/ServoConfig.h'
try:
    config = parse_servo_config(servo_config_path)
except Exception as e:
    print(f"Error parsing servo configuration: {e}")
    sys.exit(1)

# Extract constants and raise errors if any are missing
required_keys = [
    'NUM_SERVOS', 'SERVO_NAMES', 'SERVO_MIN_ANGLES', 'SERVO_MAX_ANGLES',
    'SERVO_INVERT_MASK', 'SERVO_DEFAULT_ANGLES', 'SERVO_MIN_DEGREE',
    'SERVO_MAX_DEGREE', 'SERVO_MIN_PULSE_WIDTH', 'SERVO_MAX_PULSE_WIDTH',
    'I2C_ADDRESS_NANO', 'I2C_ADDRESS_PWMDRV'
]

for key in required_keys:
    if key not in config:
        raise KeyError(f"Missing required configuration key: {key}")

NUM_SERVOS = config['NUM_SERVOS']
SERVO_NAMES = config['SERVO_NAMES']
SERVO_MIN_ANGLES = config['SERVO_MIN_ANGLES']
SERVO_MAX_ANGLES = config['SERVO_MAX_ANGLES']
SERVO_INVERT_MASK = config['SERVO_INVERT_MASK']
SERVO_DEFAULT_ANGLES = config['SERVO_DEFAULT_ANGLES']
SERVO_MIN_DEGREE = config['SERVO_MIN_DEGREE']
SERVO_MAX_DEGREE = config['SERVO_MAX_DEGREE']
SERVO_MIN_PULSE_WIDTH = config['SERVO_MIN_PULSE_WIDTH']
SERVO_MAX_PULSE_WIDTH = config['SERVO_MAX_PULSE_WIDTH']
I2C_ADDRESS_NANO = config['I2C_ADDRESS_NANO']
I2C_ADDRESS_PWMDRV = config['I2C_ADDRESS_PWMDRV']

# Initialize baud rates after removing BAUD_RATE
BAUD_RATE_UNO = 115200
BAUD_RATE_NANO = 57600

# --------------------------------------------
# Initialize Serial Connections
# --------------------------------------------

ser_uno = None  # Serial connection to Arduino Uno
ser_nano = None  # Serial connection to Arduino Nano
serial_lock = threading.Lock()  # To synchronize access to serial ports

# Initialize Calibration
calibration = angle_calibration.AngleCalibration()

# Remove the initialization of servo_data_deques
# servo_data_deques = [collections.deque(maxlen=5) for _ in range(NUM_SERVOS)]

# --------------------------------------------
# Custom Time Axis for Displaying mm:ss
# --------------------------------------------

class TimeAxisItem(pg.AxisItem):
    """
    Custom AxisItem to display time in MM:SS format since connection started.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def tickStrings(self, values, scale, spacing):
        return [time.strftime('%M:%S', time.gmtime(v)) for v in values]

# --------------------------------------------
# Create the PyQtGraph Window
# --------------------------------------------

app = QtWidgets.QApplication([])
main_window = QtWidgets.QMainWindow()
main_window.setWindowTitle('Live Servo Feedback')

# Central widget and main layout with horizontal splitter
central_widget = QtWidgets.QWidget()
main_layout = QtWidgets.QHBoxLayout()
central_widget.setLayout(main_layout)
main_window.setCentralWidget(central_widget)

# Splitter to divide plots and terminal interface
splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
main_layout.addWidget(splitter)

# Left side: Servo plots
plots_widget = QtWidgets.QWidget()
plots_layout = QtWidgets.QVBoxLayout()
plots_widget.setLayout(plots_layout)
splitter.addWidget(plots_widget)

# Right side: Terminal interface
terminal_widget = QtWidgets.QWidget()
terminal_widget.setFixedWidth(300)
terminal_layout = QtWidgets.QVBoxLayout()
terminal_widget.setLayout(terminal_layout)
splitter.addWidget(terminal_widget)

# Adjust splitter sizes to make terminal widget twice as wide
splitter.setStretchFactor(0, 1)  # Servo plots take less space
splitter.setStretchFactor(1, 2)  # Terminal interface takes twice as much space

# --------------------------------------------
# Connect/Disconnect Button
# --------------------------------------------

connect_button = QtWidgets.QPushButton("Connect")
terminal_layout.addWidget(connect_button)

# --------------------------------------------
# Create PyQtGraph plots and labels
# --------------------------------------------

plots = []
curves = []
labels = []
pen_colors = ['r', 'g', 'b', 'c', 'm']  # Different colors for each servo
data_buffers = [[] for _ in range(NUM_SERVOS)]  # Initialize buffer for each servo

for i in range(NUM_SERVOS):
    # Horizontal layout for plot and label
    servo_layout = QtWidgets.QHBoxLayout()
    
    # Plot Widget with custom TimeAxisItem
    plot_widget = pg.PlotWidget(
        title=f'{SERVO_NAMES[i].capitalize()} Position',
        axisItems={'bottom': TimeAxisItem(orientation='bottom')}
    )
    plot_widget.setLabel('left', 'V')  # Keep label as 'V' for volts
    plot_widget.setLabel('bottom', 'Elapsed Time', units='s')
    #plot_widget.setYRange(0, 5)  # Assuming voltage ranges from 0 to 5V
    plot_widget.showGrid(x=True, y=True)  # Show grid
    curve = plot_widget.plot(pen=pen_colors[i], name=f'{SERVO_NAMES[i].capitalize()}')
    plots.append(plot_widget)
    curves.append(curve)
    
    # Current Value Label
    current_value_label = QtWidgets.QLabel(f"{SERVO_NAMES[i]}: N/A")
    current_value_label.setFixedWidth(100)
    labels.append(current_value_label)
    
    # Add plot and label to servo layout
    servo_layout.addWidget(plot_widget)
    servo_layout.addWidget(current_value_label)
    
    # Add servo layout to plots layout
    plots_layout.addLayout(servo_layout)

# --------------------------------------------
# Terminal Command Handling Widgets

# Add a QTabWidget for UNO and NANO monitors
terminal_tabs = QtWidgets.QTabWidget()
terminal_layout.addWidget(terminal_tabs)

# Debug Toggle Button

debug_toggle = QtWidgets.QCheckBox("Debug Mode")
terminal_layout.addWidget(debug_toggle)

# Initialize debug mode flag
debug_mode = False

def toggle_debug_mode(state):
    global debug_mode
    debug_mode = bool(state)
    if debug_mode:
        append_output("Debug Mode Enabled.")
    else:
        append_output("Debug Mode Disabled.")

# Connect the debug toggle button
debug_toggle.stateChanged.connect(toggle_debug_mode)

# Append Output Function

def append_output(text, device='UNO'):
    if device == 'UNO':
        uno_output_display.append(text)
    elif device == 'NANO':
        nano_output_display.append(text)
    else:
        uno_output_display.append(text)

# Calibration Controls

# Calibration Title Label
calibration_label = QtWidgets.QLabel("<b>Calibration Controls</b>")
terminal_layout.addWidget(calibration_label)

# Servo Selection Dropdown
servo_select_label = QtWidgets.QLabel("Select Servo:")
terminal_layout.addWidget(servo_select_label)

servo_select = QtWidgets.QComboBox()
servo_select.addItems(SERVO_NAMES)  # ['base', 'shoulder', 'elbow', 'wrist', 'claw']
terminal_layout.addWidget(servo_select)

# Start Calibration Button
start_calib_button = QtWidgets.QPushButton("Start Calibration")
terminal_layout.addWidget(start_calib_button)

# Stop Calibration Button
stop_calib_button = QtWidgets.QPushButton("Stop Calibration")
terminal_layout.addWidget(stop_calib_button)
stop_calib_button.setEnabled(False)  # Disabled until calibration starts

# Plot Calibration Button
plot_calib_button = QtWidgets.QPushButton("Plot Calibration Data")
terminal_layout.addWidget(plot_calib_button)

# --------------------------------------------
# Toggle Control Mode
# --------------------------------------------

# Add Toggle Button for Control Mode
control_mode_layout = QtWidgets.QHBoxLayout()
terminal_layout.addLayout(control_mode_layout)

control_mode_label = QtWidgets.QLabel("Control Mode:")
control_mode_layout.addWidget(control_mode_label)

control_mode_toggle = QtWidgets.QCheckBox("Follow Potentiometers")
control_mode_layout.addWidget(control_mode_toggle)

# Initialize Control Mode Flag
control_mode_follow = False

def toggle_control_mode(state):
    global control_mode_follow
    control_mode_follow = state
    if control_mode_follow:
        append_output("Control Mode: Potentiometers (Automatic)")
    else:
        append_output("Control Mode: Manual Sliders")

# Connect the toggle button to the function
control_mode_toggle.stateChanged.connect(toggle_control_mode)

# --------------------------------------------
# Create Servo Control Sliders
# --------------------------------------------

# Servo Control Panel
servo_control_widget = QtWidgets.QWidget()
servo_control_layout = QtWidgets.QVBoxLayout()
servo_control_widget.setLayout(servo_control_layout)
terminal_layout.addWidget(QtWidgets.QLabel("<b>Servo Controls</b>"))

# Dictionary to hold sliders
servo_sliders = {}

# Import ServoConfig if necessary or define the ranges here
# Assuming the Python interface has access to the servo angle limits

# Function to handle slider changes with inversion
def slider_changed(servo_index, degree):
    if control_mode_follow:
        # In Follow Mode, do not send commands from sliders
        return
    servo_name = SERVO_NAMES[servo_index]
    command = f"{servo_name} {degree}"
    with serial_lock:
        if ser_uno and ser_uno.is_open:
            ser_uno.write((command + '\n').encode('utf-8'))
            append_output(f"> {command}", 'UNO')
        else:
            append_output("Serial port is not connected. Please connect first.", 'UNO')

# Create Servo Control Sliders with constrained ranges
for i in range(NUM_SERVOS):
    servo_name = SERVO_NAMES[i].capitalize()
    
    # Horizontal layout for slider and label
    slider_layout = QtWidgets.QHBoxLayout()
    
    # Label for servo
    label = QtWidgets.QLabel(f"{servo_name}:")
    label.setFixedWidth(60)
    
    # Slider setup with constrained range
    slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    slider.setMinimum(SERVO_MIN_ANGLES[i])
    slider.setMaximum(SERVO_MAX_ANGLES[i])
    slider.setValue(SERVO_DEFAULT_ANGLES[i])
    slider.setTickInterval(10)
    slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
    slider.setSingleStep(1)
    slider.setPageStep(10)
    
    # Current value label
    slider_value_label = QtWidgets.QLabel(f"{SERVO_DEFAULT_ANGLES[i]}°")
    slider_value_label.setFixedWidth(40)
    
    # Connect slider's valueChanged signal
    slider.valueChanged.connect(lambda value, idx=i, lbl=slider_value_label: (lbl.setText(f"{value}°"), slider_changed(idx, value)))
    
    # Add widgets to slider layout
    slider_layout.addWidget(label)
    slider_layout.addWidget(slider)
    slider_layout.addWidget(slider_value_label)
    
    # Add slider layout to servo control layout
    servo_control_layout.addLayout(slider_layout)
    
    # Store slider in dictionary
    servo_sliders[i] = slider

# --------------------------------------------
# Add "Reset Sliders" Button
# --------------------------------------------

reset_sliders_button = QtWidgets.QPushButton("Reset Positions")
terminal_layout.addWidget(reset_sliders_button)

def reset_sliders():
    for i in range(NUM_SERVOS):
        default_angle = SERVO_DEFAULT_ANGLES[i]
        servo_sliders[i].blockSignals(True)  # Prevent triggering slider_changed
        servo_sliders[i].setValue(default_angle)
        servo_sliders[i].blockSignals(False)
        # Update the corresponding label
        # Retrieve the label from the layout
        slider_layout = servo_control_layout.itemAt(i).layout()
        slider_label = slider_layout.itemAt(2).widget()
        slider_label.setText(f"{default_angle}°")
        if not control_mode_follow:
            # Send the command to the servo only if not in Follow Mode
            command = f"{SERVO_NAMES[i]} {default_angle}"
            with serial_lock:
                if ser_uno and ser_uno.is_open:
                    ser_uno.write((command + '\n').encode('utf-8'))
                    append_output(f"Resetting {SERVO_NAMES[i]} to {default_angle}°")

reset_sliders_button.clicked.connect(reset_sliders)

# Add Servo Control Panel to terminal layout
terminal_layout.addWidget(servo_control_widget)

# --------------------------------------------
# Function to Toggle Serial Connections
# --------------------------------------------

def toggle_connection():
    global ser_uno, ser_nano, connection_start_time
    if ser_uno and ser_uno.is_open and ser_nano and ser_nano.is_open:
        # Disconnect both
        with serial_lock:
            ser_uno.close()
            ser_nano.close()
            ser_uno = None
            ser_nano = None
        connect_button.setText("Connect")
        append_output("Disconnected from both serial ports.")
    else:
        # Connect both
        try:
            with serial_lock:
                ser_uno = serial.Serial(SERIAL_PORT_UNO, BAUD_RATE_UNO, timeout=1)
                ser_nano = serial.Serial(SERIAL_PORT_NANO, BAUD_RATE_NANO, timeout=1)
            connect_button.setText("Disconnect")
            append_output(f"Connected to Uno at {SERIAL_PORT_UNO} and Nano at {SERIAL_PORT_NANO}.")
            connection_start_time = time.time()
            # Clear data buffers upon connection
            for buffer in data_buffers:
                buffer.clear()
            # Reset all current value labels
            for label in labels:
                label.setText("N/A")
        except serial.SerialException as e:
            append_output(f"Failed to connect: {e}")
            ser_uno = None
            ser_nano = None

# Connect the button to the toggle function
connect_button.clicked.connect(toggle_connection)

# Modify UNO Tab to Include Command Input Field

# Create a widget and layout for the UNO tab
uno_tab_widget = QtWidgets.QWidget()
uno_tab_layout = QtWidgets.QVBoxLayout()
uno_tab_widget.setLayout(uno_tab_layout)

# UNO Output Display
uno_output_display = QtWidgets.QTextEdit()
uno_output_display.setReadOnly(True)
uno_output_display.setPlaceholderText("UNO Monitor...")
uno_tab_layout.addWidget(uno_output_display)

# UNO Command Input Field
uno_command_input = QtWidgets.QLineEdit()
uno_command_input.setPlaceholderText("Enter command here...")
uno_tab_layout.addWidget(uno_command_input)

# Add the UNO tab to the terminal tabs
terminal_tabs.addTab(uno_tab_widget, "UNO Monitor")

# Ensure that send_uno_command also accounts for inversion
def send_uno_command():
    user_input = uno_command_input.text().strip()
    if user_input:
        with serial_lock:
            if ser_uno and ser_uno.is_open:
                try:
                    # Expected command format: "servo_name degrees", e.g., "base 90"
                    parts = user_input.split()
                    if len(parts) == 2:
                        servo, degrees = parts
                        if servo in SERVO_NAMES:
                            try:
                                degrees = float(degrees)
                                if not (SERVO_MIN_DEGREE <= degrees <= SERVO_MAX_DEGREE):
                                    append_output(f"Degrees must be between {SERVO_MIN_DEGREE} and {SERVO_MAX_DEGREE}.", 'UNO')
                                    return


                                # Send degrees to Arduino Uno
                                command = f"{servo} {degrees}"
                                ser_uno.write((command + '\n').encode('utf-8'))

                                append_output(f"> {command}", 'UNO')
                                uno_command_input.clear()

                            except ValueError:
                                append_output("Invalid degrees value. Please enter a numeric value.", 'UNO')
                        else:
                            append_output(f"Unknown servo name: {servo}", 'UNO')
                    else:
                        append_output("Invalid command format. Use 'servo_name degrees', e.g., 'base 90'", 'UNO')
                except serial.SerialException as e:
                    append_output(f"Error sending command: {e}", 'UNO')
            else:
                append_output("Serial port is not connected. Please connect first.", 'UNO')

# Connect the UNO command input field to 'send_uno_command'
uno_command_input.returnPressed.connect(send_uno_command)

# Modify NANO Tab Layout

# Create a widget and layout for the NANO tab
nano_tab_widget = QtWidgets.QWidget()
nano_tab_layout = QtWidgets.QVBoxLayout()
nano_tab_widget.setLayout(nano_tab_layout)

# NANO Output Display
nano_output_display = QtWidgets.QTextEdit()
nano_output_display.setReadOnly(True)
nano_output_display.setPlaceholderText("NANO Monitor...")
nano_tab_layout.addWidget(nano_output_display)

# Add the NANO tab to the terminal tabs
terminal_tabs.addTab(nano_tab_widget, "NANO Monitor")

# Calibration Functionality

def start_calibration():
    selected_servo = servo_select.currentText()
    calibration.start_logging()
    uno_output_display.append(f"Calibration started for servo: {selected_servo}")
    start_calib_button.setEnabled(False)
    stop_calib_button.setEnabled(True)

def stop_calibration():
    calibration.stop_logging()
    uno_output_display.append("Calibration stopped.")
    start_calib_button.setEnabled(True)
    stop_calib_button.setEnabled(False)

def plot_calibration():
    selected_servo = servo_select.currentText()
    calibration.plot_data(selected_servo)
    uno_output_display.append(f"Plotted calibration data for servo: {selected_servo}")

# Connect calibration buttons to their functions
start_calib_button.clicked.connect(start_calibration)
stop_calib_button.clicked.connect(stop_calibration)
plot_calib_button.clicked.connect(plot_calibration)

# --------------------------------------------
# Function to Handle Incoming Serial Data
# --------------------------------------------

# Mapping from raw analog values to degrees
def analog_to_degree(raw, servo_index):
    """
    Convert raw analog value to degrees based on a fixed mapping.
    """
    # Define calibration parameters (unchanged)
    ANALOG_MIN = 32
    ANALOG_MAX = 1023

    # Clamp the raw value to the expected range
    raw = max(ANALOG_MIN, min(raw, ANALOG_MAX))

    # Linear mapping from raw value to degrees
    degree = ((ANALOG_MAX - raw) / (ANALOG_MAX - ANALOG_MIN)) * (SERVO_MAX_ANGLES[servo_index] - SERVO_MIN_ANGLES[i]) + SERVO_MIN_ANGLES[servo_index]
    return int(degree)

# Read NANO serial, write to UNO serial
def update():
    if ser_nano and ser_nano.is_open and ser_nano.in_waiting > 0:
        try:
            line = ser_nano.readline().decode('utf-8').strip()
            if debug_mode:
                append_output(line, 'NANO')
            if line.startswith('>') or debug_mode:
                if line.startswith('>'):
                    data = line[1:]
                else:
                    data = line
                raw_values = list(map(float, data.split(',')))
                if len(raw_values) == NUM_SERVOS:
                    if control_mode_follow:
                        # Append new raw values to the respective deques
                        for i in range(NUM_SERVOS):
                            raw = raw_values[i]
                            degree = analog_to_degree(raw, i)
                            servo_name = SERVO_NAMES[i]
                            command = f"{servo_name} {degree}"
                            with serial_lock:
                                if ser_uno and ser_uno.is_open:
                                    ser_uno.write((command + '\n').encode('utf-8'))
                                    append_output(f"> {command}", 'UNO')
                            servo_sliders[i].blockSignals(True)
                            servo_sliders[i].setValue(degree)
                            servo_sliders[i].blockSignals(False)
                else:
                    if debug_mode:
                        append_output(f"Malformed data: {line}", 'NANO')
        except ValueError:
            append_output(f"Nano: {line}")
        except serial.SerialException as e:
            append_output(f"Serial error with Nano: {e}")
            toggle_connection()

    if ser_uno and ser_uno.is_open and ser_uno.in_waiting > 0:
        try:
            line = ser_uno.readline().decode('utf-8').strip()
            if debug_mode:
                append_output(line, 'UNO')
            if line.startswith('>') or debug_mode:
                if line.startswith('>'):
                    data = line[1:]
                else:
                    data = line
                values = list(map(float, data.split(',')))
                if len(values) == NUM_SERVOS:
                    current_time = time.time() - connection_start_time
                    for i in range(NUM_SERVOS):
                        voltage = values[i]
                        data_buffers[i].append((current_time, voltage))
                        # Update plot data
                        times, voltages = zip(*data_buffers[i][-100:])
                        curves[i].setData(times, voltages)
                        # Update current value label
                        labels[i].setText(f"{SERVO_NAMES[i]}: {voltage:.2f} V")
                else:
                    if debug_mode:
                        append_output(f"Malformed data: {line}", 'UNO')
            else:
                append_output(line)
        except ValueError:
            append_output(f"Uno: {line}")
        except serial.SerialException as e:
            append_output(f"Serial error with Uno: {e}")
            toggle_connection()

# Timer to update plots and handle serial data
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)  # Update every 100 ms

# --------------------------------------------
# Signal Handling for Graceful Exit
# --------------------------------------------

# Function to reset servos to default positions slowly
def reset_servos():
    with serial_lock:
        if ser_uno and ser_uno.is_open:
            append_output("Resetting servos to default positions...", 'UNO')
            for i in range(NUM_SERVOS):
                servo_name = SERVO_NAMES[i]
                default_angle = SERVO_DEFAULT_ANGLES[i]
                command = f"{servo_name} {default_angle}"
                try:
                    ser_uno.write((command + '\n').encode('utf-8'))
                    append_output(f"Resetting {servo_name} to {default_angle}°", 'UNO')
                    # Update sliders and labels
                    if i in servo_sliders:
                        servo_sliders[i].blockSignals(True)
                        servo_sliders[i].setValue(default_angle)
                        servo_sliders[i].blockSignals(False)
                        slider_layout = servo_control_layout.itemAt(i).layout()
                        slider_label = slider_layout.itemAt(2).widget()
                        slider_label.setText(f"{default_angle}°")
                    time.sleep(0.5)  # Delay for smooth motion
                except serial.SerialException as e:
                    append_output(f"Failed to reset {servo_name}: {e}", 'UNO')

# Register the reset_servos function to be called on exit
atexit.register(reset_servos)

# Modify the signal_handler to call reset_servos before exiting
def signal_handler(sig, frame):
    append_output("\nKeyboardInterrupt detected. Exiting gracefully...")
    reset_servos()
    if ser_uno and ser_uno.is_open:
        with serial_lock:
            ser_uno.close()
            append_output("Serial port closed.")
    QtWidgets.QApplication.quit()

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# --------------------------------------------
# Start the Qt Event Loop and Auto-Connect
# --------------------------------------------

main_window.show()
toggle_connection()  # Auto-connect on startup

try:
    sys.exit(app.exec_())
except SystemExit:
    pass