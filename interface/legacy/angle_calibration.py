# angle_calibration.py


# --------------------------------------------
# LEGACY CODE:
# NOT TO BE EDITED OR USED
# --------------------------------------------



import csv
import os
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

class AngleCalibration:
    def __init__(self, filename='calibration_log.csv'):
        self.filename = filename
        self.logging = False
        self.fieldnames = ['timestamp', 'servo_name', 'input_pwm', 'output_voltage']
        self.last_input = {}  # To store the last input_pwm per servo

    def start_logging(self):
        self.logging = True
        self.last_input = {}
        # Create or overwrite the existing log file
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writeheader()
        print("Logging started.")

    def log_input(self, servo_name, input_pwm, timestamp=None):
        if not self.logging:
            return
        if timestamp is None:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.last_input[servo_name] = input_pwm
        # Log the input with output_voltage as empty
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow({
                'timestamp': timestamp,
                'servo_name': servo_name,
                'input_pwm': input_pwm,
                'output_voltage': ''
            })

    def log_output(self, servo_name, output_voltage, timestamp=None):
        if not self.logging:
            return
        if timestamp is None:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        input_pwm = self.last_input.get(servo_name, '')
        # Log the output with the corresponding input_pwm
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow({
                'timestamp': timestamp,
                'servo_name': servo_name,
                'input_pwm': input_pwm,
                'output_voltage': output_voltage
            })
        # Optionally, clear the last input after logging
        self.last_input.pop(servo_name, None)

    def stop_logging(self):
        self.logging = False
        self.last_input = {}
        print("Logging stopped.")

    def plot_data(self, servo_name, duration=None):
        if not os.path.exists(self.filename):
            print("No calibration data found.")
            return
        # Read the CSV data
        df = pd.read_csv(self.filename)
        # Filter by servo_name
        df = df[df['servo_name'] == servo_name]
        # Drop rows where output_voltage is missing
        df = df.dropna(subset=['output_voltage'])

        if duration:
            # Convert timestamp to datetime
            df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce')
            if df['timestamp'].isnull().any():
                print("Some timestamps could not be parsed. Please check the log file.")
                return
            end_time = df['timestamp'].max()
            start_time = end_time - pd.to_timedelta(duration, unit='s')
            df = df[df['timestamp'] >= start_time]
        else:
            # Convert all timestamps to datetime
            df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce')

        if df.empty:
            print("No data to plot for the specified parameters.")
            return

        # Drop rows with invalid timestamps
        df = df.dropna(subset=['timestamp'])

        if df.empty:
            print("No valid timestamp data to plot.")
            return

        # Convert input_pwm and output_voltage to numeric, handle errors
        df['input_pwm'] = pd.to_numeric(df['input_pwm'], errors='coerce')
        df['output_voltage'] = pd.to_numeric(df['output_voltage'], errors='coerce')

        # Only drop rows where output_voltage is NaN (already done), keep input_pwm as is
        # Therefore, no need to drop rows here

        if df.empty:
            print("No complete data to plot for the specified parameters.")
            return

        # Calculate elapsed time since calibration start
        calibration_start_time = df['timestamp'].min()
        df['elapsed_time'] = (df['timestamp'] - calibration_start_time).dt.total_seconds()

        # Plot input_pwm vs time and output_voltage vs time with dual Y-axes
        fig, ax1 = plt.subplots(figsize=(12, 6))

        color = 'tab:blue'
        ax1.set_xlabel('Elapsed Time (s)')
        ax1.set_ylabel('Output Voltage (V)', color=color)
        ax1.plot(df['elapsed_time'], df['output_voltage'], color=color, label='Output Voltage')
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.grid(True)
        
        # Draw vertical lines at points where input_pwm is present
        for _, row in df.dropna(subset=['input_pwm']).iterrows():
            ax1.axvline(x=row['elapsed_time'], color='gray', linestyle='--', alpha=0.5)

        ax2 = ax1.twinx()  # Instantiate a second axes that shares the same x-axis

        color = 'tab:red'
        ax2.set_ylabel('Input PWM Pulse Length', color=color)  # We already handled the x-label
        # Scatter plot only where input_pwm is present
        ax2.scatter(df['elapsed_time'], df['input_pwm'], color=color, label='Input PWM', marker='x', s=100)
        ax2.tick_params(axis='y', labelcolor=color)

        # Add legends
        lines_1, labels_1 = ax1.get_legend_handles_labels()
        lines_2, labels_2 = ax2.get_legend_handles_labels()
        ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper left')
        
        ax1.legend(loc='upper left')
        ax2.legend(loc='upper right')

        plt.title(f'Calibration Data for {servo_name.capitalize()} Servo')
        fig.tight_layout()  # Otherwise the right y-label is slightly clipped
        plt.show()