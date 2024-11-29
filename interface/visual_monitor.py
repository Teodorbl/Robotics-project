import sys
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
from collections import deque
import signal
import time  # Added import

# --------------------------------------------
# Configuration Section
# --------------------------------------------

# Configure the serial port (update '/dev/cu.usbmodemXXXX' to your Arduino's port)
SERIAL_PORT = '/dev/tty.usbmodem11101'  # Replace with your Arduino's serial port
BAUD_RATE = 9600

# Number of servos
NUM_SERVOS = 5

# --------------------------------------------
# Initialize Serial Connection
# --------------------------------------------

ser = None  # Start with no connection

# --------------------------------------------
# Create the PyQtGraph Window
# --------------------------------------------

app = QtWidgets.QApplication([])
main_window = QtWidgets.QMainWindow()
main_window.setWindowTitle('Live Servo Feedback')

# Central widget and layout
central_widget = QtWidgets.QWidget()
main_layout = QtWidgets.QVBoxLayout()
central_widget.setLayout(main_layout)
main_window.setCentralWidget(central_widget)

# Connect/Disconnect Button
connect_button = QtWidgets.QPushButton("Connect")
main_layout.addWidget(connect_button)

# Create PyQtGraph plots
plots = []
curves = []
pen_colors = ['r', 'g', 'b', 'c', 'm']  # Different colors for each servo
data = [deque(maxlen=100) for _ in range(NUM_SERVOS)]  # Store last 100 points

for i in range(NUM_SERVOS):
    plot_widget = pg.PlotWidget(title=f'Servo {i} Position')
    plot_widget.setLabel('left', 'Degrees')
    plot_widget.setLabel('bottom', 'Time', units='s')
    plot_widget.setYRange(0, 180)
    curve = plot_widget.plot(pen=pen_colors[i], name=f'Servo {i}')
    main_layout.addWidget(plot_widget)
    plots.append(plot_widget)
    curves.append(curve)

# --------------------------------------------
# Data Buffers for Plotting
# --------------------------------------------

max_points = 100  # Maximum number of points to display
data_buffers = [[] for _ in range(NUM_SERVOS)]  # Initialize buffer for each servo

# --------------------------------------------
# Function to Toggle Serial Connection
# --------------------------------------------

def toggle_connection():
    global ser
    if ser and ser.is_open:
        # Disconnect
        ser.close()
        ser = None
        connect_button.setText("Connect")
        print("Disconnected from serial port.")
    else:
        # Connect
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            connect_button.setText("Disconnect")
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            # Clear data buffers upon connection
            for buffer in data_buffers:
                buffer.clear()
        except serial.SerialException as e:
            print(f"Failed to connect to {SERIAL_PORT}: {e}")

# Connect the button to the toggle function
connect_button.clicked.connect(toggle_connection)

# --------------------------------------------
# Update Function
# --------------------------------------------

def update():
    if ser and ser.is_open and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Expecting a comma-separated string of degrees, e.g., "90,45,135,60,120"
                values = list(map(float, line.split(',')))
                if len(values) == NUM_SERVOS:
                    current_time = time.time()  # Replaced pg.ptime.time() with time.time()
                    for i in range(NUM_SERVOS):
                        data_buffers[i].append((current_time, values[i]))
                        # Maintain buffer size
                        if len(data_buffers[i]) > max_points:
                            data_buffers[i].pop(0)
                        # Extract x and y data
                        x, y = zip(*data_buffers[i])
                        # Adjust time to start from zero
                        x = [t - data_buffers[i][0][0] for t in x]
                        curves[i].setData(x, y)
                else:
                    print(f"Received incorrect number of servo degrees: {values}")
        except ValueError:
            print(f"Received malformed data: {line}")
    # Optional: Add a small delay if necessary
    # QtCore.QThread.msleep(10)

# Timer to update plots
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)  # Update every 100 ms

# --------------------------------------------
# Signal Handling for Graceful Exit
# --------------------------------------------

def signal_handler(sig, frame):
    print("\nKeyboardInterrupt detected. Exiting gracefully...")
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")
    QtWidgets.QApplication.quit()

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# --------------------------------------------
# Start the Qt Event Loop
# --------------------------------------------

main_window.show()

try:
    sys.exit(app.exec_())
except SystemExit:
    pass