# FILE: CURRENT_HISTOGRAMS.PY

import pandas as pd
import matplotlib.pyplot as plt

# Read the log file
log_file = "1command_feedback_log.csv"
df = pd.read_csv(log_file)

# Filter rows where event_type is 'feedback_received' and content starts with 'Uno_FB'
uno_feedback = df[
    (df['event_type'] == 'feedback_received') &
    (df['content'].str.startswith("Uno_FB"))
].copy()

# Extract timestamp and servo3_feedback from 'content'
# Assuming 'Uno_FB' format: "Uno_FB: value1,value2,value3,value4"
uno_feedback[['servo1_fb', 'servo2_fb', 'servo3_fb', 'servo4_fb']] = uno_feedback['content'].str.extract(
    r'Uno_FB:\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)'
)

# Convert extracted feedbacks to numeric
uno_feedback['servo3_fb'] = pd.to_numeric(uno_feedback['servo3_fb'], errors='coerce')
uno_feedback['timestamp'] = pd.to_numeric(uno_feedback['timestamp'], errors='coerce')

# Drop rows with NaN values in 'servo3_fb' or 'timestamp'
uno_feedback.dropna(subset=['servo3_fb', 'timestamp'], inplace=True)

# Sort by timestamp
uno_feedback.sort_values('timestamp', inplace=True)

# Reset index
uno_feedback.reset_index(drop=True, inplace=True)

# Compute angular speed: (delta_servo3_fb) / (delta_time)
uno_feedback['delta_servo3_fb'] = uno_feedback['servo3_fb'].diff()
uno_feedback['delta_time'] = uno_feedback['timestamp'].diff()

# Calculate angular speed
uno_feedback['angular_speed'] = uno_feedback['delta_servo3_fb'] / uno_feedback['delta_time']

# Drop the first row as it will have NaN values for diffs
uno_feedback.dropna(subset=['angular_speed'], inplace=True)

# Plot angular speed over time
plt.figure(figsize=(12, 6))
plt.plot(uno_feedback['timestamp'], uno_feedback['angular_speed'], marker='o', linestyle='-')
plt.title('Angular Speed of Servo 3 Over Time')
plt.xlabel('Timestamp (s)')
plt.ylabel('Angular Speed (Analog Units/s)')
plt.grid(True)
plt.tight_layout()
plt.show()

# Optional: Display the first few rows of the data
print("Servo 3 Angular Speed Over Time:")
print(uno_feedback[['timestamp', 'servo3_fb', 'delta_servo3_fb', 'delta_time', 'angular_speed']].head())