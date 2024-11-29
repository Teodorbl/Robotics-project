# FILE:FULL_INTERFACE.PY

import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import signal
import threading
import time
import angle_calibration  # Import the calibration module
from datetime import datetime

# --------------------------------------------
# Configuration Section
# --------------------------------------------

# Configure the serial port (update '/dev/cu.usbmodemXXXX' to your Arduino's port)
SERIAL_PORT = '/dev/tty.usbmodem11101'  # Replace with your Arduino's serial port
BAUD_RATE = 9600

# Number of servos
NUM_SERVOS = 5

# Servo Names
SERVO_NAMES = ['base', 'shoulder', 'elbow', 'wrist', 'claw']

# Initialize servo angles
servoAngles = [90, 50, 45, 90, 120]  # Initial angles for each servo

# Mapping from degrees to PWM pulse
def degrees_to_pwm(degrees):
    """
    Convert degrees to PWM pulse length based on linear mapping [0, 180] -> [184, 430].
    """
    pwm_min = 184
    pwm_max = 430
    degrees_min = 0
    degrees_max = 180
    pwm = int((degrees - degrees_min) * (pwm_max - pwm_min) / (degrees_max - degrees_min) + pwm_min)
    return pwm

# --------------------------------------------
# Initialize Serial Connection
# --------------------------------------------

ser = None  # Start with no connection
serial_lock = threading.Lock()  # To synchronize access to serial port
connection_start_time = None  # To track when the connection was established

# Initialize Calibration
calibration = angle_calibration.AngleCalibration()

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
    plot_widget.setLabel('left', 'V')  # Changed label from 'Degrees' to 'V' for volts
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
# --------------------------------------------

# Output Display Area
output_display = QtWidgets.QTextEdit()
output_display.setReadOnly(True)
output_display.setPlaceholderText("Terminal Output...")
terminal_layout.addWidget(output_display)

# Command Input Field
command_input = QtWidgets.QLineEdit()
command_input.setPlaceholderText("Enter command here...")
terminal_layout.addWidget(command_input)

# --------------------------------------------
# Calibration Controls
# --------------------------------------------

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
# Create Servo Control Sliders
# --------------------------------------------

# Servo Control Panel
servo_control_widget = QtWidgets.QWidget()
servo_control_layout = QtWidgets.QVBoxLayout()
servo_control_widget.setLayout(servo_control_layout)
terminal_layout.addWidget(QtWidgets.QLabel("<b>Servo Controls</b>"))

# Dictionary to hold sliders
servo_sliders = {}

def slider_changed(servo_index, value):
    global servoAngles
    servo_name = SERVO_NAMES[servo_index]
    degree = value
    command = f"{servo_name} {degree}"
    with serial_lock:
        if ser and ser.is_open:
            ser.write((command + '\n').encode('utf-8'))
            append_output(f"> {command}")
            # Update servoAngles if needed
            servoAngles[servo_index] = degree
            # Log if calibration is active
            if calibration.logging:
                pwm = degrees_to_pwm(degree)
                calibration.log_input(servo_name=servo_name, input_pwm=pwm)
                output_display.append(f"Logged input for servo '{servo_name}': {pwm} PWM, {degree} deg")
        else:
            append_output("Serial port is not connected. Please connect first.")

for i in range(NUM_SERVOS):
    servo_name = SERVO_NAMES[i].capitalize()
    
    # Horizontal layout for slider and label
    slider_layout = QtWidgets.QHBoxLayout()
    
    # Label for servo
    label = QtWidgets.QLabel(f"{servo_name}:")
    label.setFixedWidth(60)
    
    # Slider setup
    slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    slider.setMinimum(0)
    slider.setMaximum(180)
    slider.setValue(servoAngles[i])
    slider.setTickInterval(10)
    slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
    slider.setSingleStep(1)
    slider.setPageStep(10)
    
    # Current value label
    slider_value_label = QtWidgets.QLabel(f"{servoAngles[i]}°")
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

reset_sliders_button = QtWidgets.QPushButton("Reset Sliders")
terminal_layout.addWidget(reset_sliders_button)

def reset_sliders():
    for i in range(NUM_SERVOS):
        initial_angle = servoAngles[i]
        servo_sliders[i].blockSignals(True)  # Prevent triggering slider_changed
        servo_sliders[i].setValue(initial_angle)
        servo_sliders[i].blockSignals(False)
        # Update the corresponding label
        # Retrieve the label from the layout
        slider_layout = servo_control_layout.itemAt(i).layout()
        slider_label = slider_layout.itemAt(2).widget()
        slider_label.setText(f"{initial_angle}°")
        # Send the command to the servo
        slider_changed(i, initial_angle)

reset_sliders_button.clicked.connect(reset_sliders)

# Add Servo Control Panel to terminal layout
terminal_layout.addWidget(servo_control_widget)

# --------------------------------------------
# Function to Toggle Serial Connection
# --------------------------------------------

def toggle_connection():
    global ser, connection_start_time
    if ser and ser.is_open:
        # Disconnect
        with serial_lock:
            ser.close()
            ser = None
            connection_start_time = None
        connect_button.setText("Connect")
        append_output("Disconnected from serial port.")
    else:
        # Connect
        try:
            with serial_lock:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            connect_button.setText("Disconnect")
            append_output(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            # Record the connection start time
            connection_start_time = time.time()
            # Clear data buffers upon connection
            for buffer in data_buffers:
                buffer.clear()
            # Reset all current value labels
            for label in labels:
                label.setText("N/A")
        except serial.SerialException as e:
            append_output(f"Failed to connect to {SERIAL_PORT}: {e}")

# Connect the button to the toggle function
connect_button.clicked.connect(toggle_connection)

# --------------------------------------------
# Function to Append Text to Output Display
# --------------------------------------------

def append_output(text):
    if text.startswith('>'):
        return
    output_display.append(text)

# --------------------------------------------
# Function to Handle Command Input
# --------------------------------------------

def send_command():
    user_input = command_input.text().strip()
    if user_input:
        with serial_lock:
            if ser and ser.is_open:
                try:
                    # Expected command format: "servo_name degrees", e.g., "base 90"
                    parts = user_input.split()
                    if len(parts) == 2:
                        servo, degrees = parts
                        if servo in SERVO_NAMES:
                            try:
                                degrees = float(degrees)
                                if not (0 <= degrees <= 180):
                                    append_output("Degrees must be between 0 and 180.")
                                    return
                                
                                # Map degrees to PWM for logging
                                pwm = degrees_to_pwm(degrees)
                                
                                # **Send Degrees to Arduino**
                                command = f"{servo} {degrees}"
                                ser.write((command + '\n').encode('utf-8'))
                                
                                append_output(f"> {command}")
                                command_input.clear()
                                
                                # **Log PWM Value if Calibration is Active**
                                if calibration.logging:
                                    calibration.log_input(servo_name=servo, input_pwm=pwm)
                                    output_display.append(f"Logged input for servo '{servo}': {pwm} PWM, {degrees} deg")
                            except ValueError:
                                append_output("Invalid degrees value. Please enter a numeric value.")
                        else:
                            append_output(f"Unknown servo name: {servo}")
                    else:
                        append_output("Invalid command format. Use 'servo_name degrees', e.g., 'base 90'")
                except serial.SerialException as e:
                    append_output(f"Error sending command: {e}")
            else:
                append_output("Serial port is not connected. Please connect first.")

# Connect the command_input's returnPressed signal to send_command
command_input.returnPressed.connect(send_command)

# --------------------------------------------
# Calibration Functionality
# --------------------------------------------

def start_calibration():
    selected_servo = servo_select.currentText()
    calibration.start_logging()
    output_display.append(f"Calibration started for servo: {selected_servo}")
    start_calib_button.setEnabled(False)
    stop_calib_button.setEnabled(True)

def stop_calibration():
    calibration.stop_logging()
    output_display.append("Calibration stopped.")
    start_calib_button.setEnabled(True)
    stop_calib_button.setEnabled(False)

def plot_calibration():
    selected_servo = servo_select.currentText()
    # Optionally, prompt user for duration or use all data
    # For simplicity, we'll plot all logged data for the selected servo
    calibration.plot_data(selected_servo)
    output_display.append(f"Plotted calibration data for servo: {selected_servo}")

# Connect calibration buttons to their functions
start_calib_button.clicked.connect(start_calibration)
stop_calib_button.clicked.connect(stop_calibration)
plot_calib_button.clicked.connect(plot_calibration)

# --------------------------------------------
# Function to Handle Incoming Serial Data
# --------------------------------------------

def update():
    if ser and ser.is_open and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                if line.startswith('>'):
                    # Remove the '>' prefix before splitting
                    data = line[1:]

                    # Split the data by commas and convert to floats
                    values = list(map(float, data.split(',')))

                    if len(values) == NUM_SERVOS:
                        current_time = time.time()
                        elapsed_time = current_time - connection_start_time

                        for i in range(NUM_SERVOS):
                            servo_name = SERVO_NAMES[i]
                            output_voltage = values[i]

                            data_buffers[i].append((elapsed_time, output_voltage))

                            # Update current value label
                            labels[i].setText(f"{servo_name}: {output_voltage:.2f} V")

                            # Maintain buffer size
                            if len(data_buffers[i]) > 100:
                                data_buffers[i].pop(0)

                            # Extract x and y data for plotting
                            x, y = zip(*data_buffers[i])
                            curves[i].setData(x, y)

                            # Adjust time range to show only the last 100 points
                            if len(x) >= 2:
                                plots[i].setXRange(x[0], x[-1])

                            # If calibration is active, log the output_voltage
                            if calibration.logging:
                                calibration.log_output(servo_name=servo_name, output_voltage=output_voltage)
                else:
                    # Display lines that do NOT start with '>' in the terminal
                    append_output(line)
        except ValueError:
            append_output(f"Received malformed data: {line}")
        except serial.SerialException as e:
            append_output(f"Serial error: {e}")
            toggle_connection()  # Attempt to disconnect on serial error

# Timer to update plots
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)  # Update every 100 ms

# --------------------------------------------
# Signal Handling for Graceful Exit
# --------------------------------------------

def signal_handler(sig, frame):
    append_output("\nKeyboardInterrupt detected. Exiting gracefully...")
    if ser and ser.is_open:
        with serial_lock:
            ser.close()
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