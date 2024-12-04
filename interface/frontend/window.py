from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph as pg
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from frontend.gui import GUI as GUIType

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

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        self.setWindowTitle('Servo Interface')
        # ...existing code to set up the main window...

    def closeEvent(self, event):
        # Call the GUI's close method for cleanup
        self.gui.close()
        event.accept()

class Window():
    def __init__(self, gui: 'GUIType', configs):

        # --------------------------------------------
        # Create the PyQtGraph Window
        # --------------------------------------------

        self.gui = gui  # Store a reference to the GUI
        self.app = QtWidgets.QApplication([])
        self.main_window = MainWindow(gui)

        # Central widget and main layout with horizontal splitter
        central_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(main_layout)
        self.main_window.setCentralWidget(central_widget)

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

        self.connect_button = QtWidgets.QPushButton("Connect")
        terminal_layout.addWidget(self.connect_button)
        # Connect the button to the toggle function
        self.connect_button.clicked.connect(gui.toggle_connection)

        # --------------------------------------------
        # Create PyQtGraph plots and labels
        # --------------------------------------------
        
        self.pos_curves = []
        self.pos_labels = []
        self.current_curves = []
        self.current_labels = []
        pen_colors = ['r', 'g', 'orange', 'c', 'm']

        for i in range(configs.NUM_SERVOS):
            # Horizontal layout for plot and label
            servo_layout = QtWidgets.QHBoxLayout()
            
            # Plot Widget with custom TimeAxisItem for Voltage
            pos_plot_widget = pg.PlotWidget(
                title=f'{configs.SERVO_NAMES[i].capitalize()} Position',
                axisItems={'bottom': TimeAxisItem(orientation='bottom')}
            )
            
            pos_plot_widget.setLabel('left', 'Value')
            pos_plot_widget.setLabel('bottom', 'Elapsed Time', units='s')
            
            pos_plot_widget.showGrid(x=True, y=True)  # Show grid
            pos_curve = pos_plot_widget.plot(pen=pen_colors[i], name=f'{configs.SERVO_NAMES[i].capitalize()} Angle')
            self.pos_curves.append(pos_curve)
            
            # Current Value Label
            pos_value_label = QtWidgets.QLabel(f"{configs.SERVO_NAMES[i]}: N/A")
            pos_value_label.setFixedWidth(100)
            self.pos_labels.append(pos_value_label)
            
            # Add plot and label to servo layout
            servo_layout.addWidget(pos_plot_widget)
            servo_layout.addWidget(pos_value_label)
            
            # --------------------------------------------
            # Create PyQtGraph plots and labels for Current
            # --------------------------------------------
            
            # Plot Widget for Current
            current_plot_widget = pg.PlotWidget(
                title=f'{configs.SERVO_NAMES[i].capitalize()} Current',
                axisItems={'bottom': TimeAxisItem(orientation='bottom')}
            )
            
            current_plot_widget.setLabel('left', 'Amps')
            current_plot_widget.setLabel('bottom', 'Elapsed Time', units='s')
            
            current_plot_widget.showGrid(x=True, y=True)  # Show grid
            current_curve = current_plot_widget.plot(pen=pen_colors[i])
            self.current_curves.append(current_curve)
            
            # Electrical Current Value Label
            current_value_label = QtWidgets.QLabel(f"{configs.SERVO_NAMES[i]}: N/A")
            current_value_label.setFixedWidth(100)
            self.current_labels.append(current_value_label)
            
            # Add current plot and label to servo layout
            servo_layout.addWidget(current_plot_widget)
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
        self.debug_toggle = QtWidgets.QCheckBox("Debug Mode")
        terminal_layout.addWidget(self.debug_toggle)
        
        # Connect the debug toggle button
        self.debug_toggle.stateChanged.connect(gui.toggle_debug_mode)
        
        
        # --------------------------------------------
        # Toggle Control Mode
        # --------------------------------------------

        # Add Toggle Button for Knob Mode
        knob_mode_layout = QtWidgets.QHBoxLayout()
        terminal_layout.addLayout(knob_mode_layout)

        knob_mode_label = QtWidgets.QLabel("Knob Mode:")
        knob_mode_layout.addWidget(knob_mode_label)

        self.knob_mode_toggle = QtWidgets.QCheckBox("Follow Knobs")
        knob_mode_layout.addWidget(self.knob_mode_toggle)
        
        # Connect the toggle button to the function
        self.knob_mode_toggle.stateChanged.connect(gui.toggle_knob_mode)
        
        
        # --------------------------------------------
        # Create Servo Control Sliders
        # --------------------------------------------

        # Servo Control Panel
        servo_control_widget = QtWidgets.QWidget()
        self.servo_control_layout = QtWidgets.QVBoxLayout()
        servo_control_widget.setLayout(self.servo_control_layout)
        terminal_layout.addWidget(QtWidgets.QLabel("<b>Servo Controls</b>"))
        
        # Dictionary to hold sliders
        self.servo_sliders = {}

        # Create Servo Control Sliders with constrained ranges
        for i in range(configs.NUM_SERVOS):
            servo_name = configs.SERVO_NAMES[i].capitalize()
            
            # Horizontal layout for slider and label
            slider_layout = QtWidgets.QHBoxLayout()
            
            # Label for servo
            label = QtWidgets.QLabel(f"{servo_name}:")
            label.setFixedWidth(60)
            
            # Slider setup with constrained range
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(configs.SERVO_MIN_ANGLES[i])
            slider.setMaximum(configs.SERVO_MAX_ANGLES[i])
            slider.setValue(configs.SERVO_DEFAULT_ANGLES[i])
            slider.setTickInterval(10)
            slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
            slider.setSingleStep(1)
            slider.setPageStep(10)
            
            # Current value label
            slider_value_label = QtWidgets.QLabel(f"{configs.SERVO_DEFAULT_ANGLES[i]}°")
            slider_value_label.setFixedWidth(40)
            
            # Add widgets to slider layout
            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(slider_value_label)
            
            # Add slider layout to servo control layout
            self.servo_control_layout.addLayout(slider_layout)
            
            # Store slider in dictionary
            self.servo_sliders[i] = slider
            
            # Connect slider's valueChanged signal
            self.servo_sliders[i].valueChanged.connect(
                lambda value, idx=i, lbl=slider_value_label: (lbl.setText(f"{value}°"), gui.slider_changed(idx, value))
            )

        # Reset all servos button
        reset_sliders_button = QtWidgets.QPushButton("Reset Positions")
        terminal_layout.addWidget(reset_sliders_button)
        reset_sliders_button.clicked.connect(gui.reset_sliders)
        terminal_layout.addWidget(servo_control_widget)
        
        
        # --------------------------------------------
        # UNO tab
        # --------------------------------------------

        # Create a widget and layout for the UNO tab
        uno_tab_widget = QtWidgets.QWidget()
        uno_tab_layout = QtWidgets.QVBoxLayout()
        uno_tab_widget.setLayout(uno_tab_layout)

        # UNO Output Display
        self.uno_output_display = QtWidgets.QTextEdit()
        self.uno_output_display.setReadOnly(True)
        self.uno_output_display.setPlaceholderText("UNO Monitor...")
        uno_tab_layout.addWidget(self.uno_output_display)

        # UNO Command Input Field
        self.uno_command_input = QtWidgets.QLineEdit()
        self.uno_command_input.setPlaceholderText("Enter command here...")
        uno_tab_layout.addWidget(self.uno_command_input)
        
        # Connect the UNO command input field to 'send_uno_command'
        self.uno_command_input.returnPressed.connect(self.gui.uno_text_command)

        # Add the UNO tab to the terminal tabs
        terminal_tabs.addTab(uno_tab_widget, "UNO Monitor")
                
                
        # --------------------------------------------
        # NANO tab
        # --------------------------------------------
        
        # Create a widget and layout for the NANO tab
        nano_tab_widget = QtWidgets.QWidget()
        nano_tab_layout = QtWidgets.QVBoxLayout()
        nano_tab_widget.setLayout(nano_tab_layout)

        # NANO Output Display
        self.nano_output_display = QtWidgets.QTextEdit()
        self.nano_output_display.setReadOnly(True)
        self.nano_output_display.setPlaceholderText("NANO Monitor...")
        nano_tab_layout.addWidget(self.nano_output_display)

        # Add the NANO tab to the terminal tabs
        terminal_tabs.addTab(nano_tab_widget, "NANO Monitor")
        
        # Finish construction
        self.main_window.show()





