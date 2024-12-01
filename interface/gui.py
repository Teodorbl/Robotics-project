import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import threading
import time
import signal
import atexit
from config import load_config
from serial_handler import SerialHandler
import queue  # Added import

def setup_gui(serial_handler: SerialHandler, config: dict):
    """
    Initialize and launch the PyQt GUI for the robotic arm interface.

    Args:
        serial_handler (SerialHandler): The serial handler instance for communication.
        config (dict): Configuration constants loaded from ServoConfig.h.
    """
    # Extract configuration constants
    NUM_SERVOS = config['NUM_SERVOS']
    SERVO_NAMES = config['SERVO_NAMES']
    SERVO_MIN_ANGLES = config['SERVO_MIN_ANGLES']
    SERVO_MAX_ANGLES = config['SERVO_MAX_ANGLES']
    SERVO_INVERT_MASK = config['SERVO_INVERT_MASK']
    SERVO_START_ANGLES = config['SERVO_START_ANGLES']
    SERVO_MIN_DEGREE = config['SERVO_MIN_DEGREE']
    SERVO_MAX_DEGREE = config['SERVO_MAX_DEGREE']
    SERVO_MIN_PULSE_WIDTH = config['SERVO_MIN_PULSE_WIDTH']
    SERVO_MAX_PULSE_WIDTH = config['SERVO_MAX_PULSE_WIDTH']
    I2C_ADDRESS_NANO = config['I2C_ADDRESS_NANO']
    I2C_ADDRESS_PWMDRV = config['I2C_ADDRESS_PWMDRV']

    BAUD_RATE_UNO = 115200
    BAUD_RATE_NANO = 57600

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

    # Add a QTabWidget for UNO and NANO monitors
    terminal_tabs = QtWidgets.QTabWidget()
    terminal_layout.addWidget(terminal_tabs)

    # Debug Toggle Button
    debug_toggle = QtWidgets.QCheckBox("Debug Mode")
    terminal_layout.addWidget(debug_toggle)

    # Initialize debug mode flag
    debug_mode = False

    def toggle_debug_mode(state):
        nonlocal debug_mode
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
        nonlocal control_mode_follow
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

    # Function to handle slider changes with inversion
    def slider_changed(servo_index, degree):
        if control_mode_follow:
            # In Follow Mode, do not send commands from sliders
            return
        servo_name = SERVO_NAMES[servo_index]
        command = f"{servo_name} {degree}"
        serial_handler.send_command(command, device='UNO')

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
        slider.setValue(SERVO_START_ANGLES[i])
        slider.setTickInterval(10)
        slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        slider.setSingleStep(1)
        slider.setPageStep(10)

        # Current value label
        slider_value_label = QtWidgets.QLabel(f"{SERVO_START_ANGLES[i]}°")
        slider_value_label.setFixedWidth(40)

        # Connect slider's valueChanged signal
        slider.valueChanged.connect(
            lambda value, idx=i, lbl=slider_value_label: (
                lbl.setText(f"{value}°"), slider_changed(idx, value)
            )
        )

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
            default_angle = SERVO_START_ANGLES[i]
            servo_sliders[i].blockSignals(True)  # Prevent triggering slider_changed
            servo_sliders[i].setValue(default_angle)
            servo_sliders[i].blockSignals(False)
            # Update the corresponding label
            slider_layout = servo_control_layout.itemAt(i).layout()
            slider_label = slider_layout.itemAt(2).widget()
            slider_label.setText(f"{default_angle}°")
            if not control_mode_follow:
                # Send the command to the servo only if not in Follow Mode
                command = f"{SERVO_NAMES[i]} {default_angle}"
                serial_handler.send_command(command, device='UNO')
                append_output(f"Resetting {SERVO_NAMES[i]} to {default_angle}°", 'UNO')

    reset_sliders_button.clicked.connect(reset_sliders)

    # Add Servo Control Panel to terminal layout
    terminal_layout.addWidget(servo_control_widget)

    # --------------------------------------------
    # Function to Toggle Serial Connections
    # --------------------------------------------

    def toggle_connection():
        if serial_handler.is_connected():
            serial_handler.disconnect()
            connect_button.setText("Connect")
            append_output("Disconnected from both serial ports.")
        else:
            try:
                serial_handler.connect()
                connect_button.setText("Disconnect")
                append_output(
                    f"Connected to Uno at {serial_handler.serial_uno.port} and Nano at {serial_handler.serial_nano.port}."
                )
                # Clear data buffers upon connection
                for buffer in data_buffers:
                    buffer.clear()
                # Reset all current value labels
                for label in labels:
                    label.setText("N/A")
            except Exception as e:
                append_output(f"Failed to connect: {e}", 'UNO')

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
            serial_handler.send_command(user_input, device='UNO')
            uno_command_input.clear()

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


    # --------------------------------------------
    # Function to Handle Incoming Serial Data
    # --------------------------------------------

    def update():
        # Handle incoming data from serial_handler
        serial_handler.process_incoming_data(
            append_output=append_output,
            update_plots=True,
            plots=plots,
            curves=curves,
            labels=labels,
            data_buffers=data_buffers,
            debug_mode=debug_mode,
            control_mode_follow=control_mode_follow,
            servo_sliders=servo_sliders
        )

    # Timer to update plots and handle serial data
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(100)  # Update every 100 ms

    # --------------------------------------------
    # Signal Handling for Graceful Exit
    # --------------------------------------------

    def reset_servos():
        serial_handler.reset_servos()

    # Register the reset_servos function to be called on exit
    atexit.register(reset_servos)

    # Modify the signal_handler to call reset_servos before exiting
    def signal_handler(sig, frame):
        append_output("\nKeyboardInterrupt detected. Exiting gracefully...")
        reset_servos()
        serial_handler.close_serial_ports()
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

    # Function to poll the data queue and update plots
    def poll_serial_data():
        while not serial_handler.data_queue.empty():
            try:
                values = serial_handler.data_queue.get_nowait()
                current_time = time.time()
                for i in range(config['NUM_SERVOS']):
                    # Append new data point
                    data_buffers[i].append((current_time, values[i]))
                    # Update the corresponding plot
                    times, angles = zip(*data_buffers[i][-100:])  # Keep last 100 points
                    curves[i].setData(times, angles)
                    # Update the current value label
                    labels[i].setText(f"{SERVO_NAMES[i]}: {values[i]:.2f}°")
            except queue.Empty:
                pass  # No more data to process

    # Set up a QTimer to poll the queue every 100 ms
    poll_timer = QtCore.QTimer()
    poll_timer.timeout.connect(poll_serial_data)
    poll_timer.start(100)  # Poll every 100 ms

    # Close serial ports gracefully on application exit
    def close_event():
        serial_handler.disconnect()

    main_window.closeEvent = lambda event: (close_event(), event.accept())

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
