# FILE: CURRENT_AGGREGATION.PY

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats

# Read the log file
log_file = "command_feedback_log.csv"
df = pd.read_csv(log_file)

# Convert timestamp to datetime for better readability
df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')

# Sort the DataFrame by timestamp to ensure chronological order
df = df.sort_values('timestamp').reset_index(drop=True)

# # Assign a unique command_id to each 'command_sent' event
# df['command_id'] = df['event_type'].eq('command_sent').cumsum()

# # Forward-fill the command_id for 'feedback_received' events
# df['command_id'] = df['command_id'].where(df['event_type'] == 'command_sent').ffill()

# # Separate commands and feedbacks
# commands = df[df['event_type'] == 'command_sent'].copy()
# feedbacks = df[df['event_type'] == 'feedback_received'].copy()

# # Extract servo angles from commands
# # Assuming command format: "<090,035,160,090,120>"
# commands[['servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd']] = commands['content'].str.extract(
#     r'<(\d{3}),(\d{3}),(\d{3}),(\d{3}),(\d{3})>'
# )

# # Convert extracted servo angles to integers
# for i in range(1, 6):
#     commands[f'servo{i}_cmd'] = commands[f'servo{i}_cmd'].astype(int)

# # Assign group_id for each servo based on command changes
# for i in range(1, 6):
#     servo_col = f'servo{i}_cmd'
#     # A new group is assigned when the current command differs from the previous one
#     commands[f'servo{i}_group'] = (commands[servo_col] != commands[servo_col].shift(1)).cumsum()

# # Merge commands with feedbacks based on command_id
# feedbacks = feedbacks.merge(
#     commands[['command_id', 'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
#               'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group']],
#     on='command_id',
#     how='left'
# )

# Extract Uno_Current and Nano_Servo5_Current from feedbacks
# Assuming all 5 currents are sent together by the same Arduino Nano
# Example content: "Uno_Current: 0.12,0.13,0.11,0.14,0.10"
df[['servo1_current', 'servo2_current', 'servo3_current', 'servo4_current', 'servo5_current']] = df['content'].str.extract(
    r'Nano_Current: (\d+),(\d+),(\d+),(\d+),(\d+)'
)
df = df.dropna()

servo_current_offsets = [524, 531, 530, 529, 531]

def calculate_amps(analog_read, offset):
    return 5 * (int(analog_read) - offset) / (1023 * 0.185)

# Apply the function to each servo_current column
for i in range(1, 6):
    servo_col = f'servo{i}_current'
    offset = servo_current_offsets[i-1]
    df[servo_col] = df[servo_col].apply(lambda x: calculate_amps(x, offset))

# Assuming df is your DataFrame and it contains the current values for each servo
plt.figure(figsize=(10, 6))

# Plotting each servo's current
plt.plot(df['servo1_current'], label='Servo 1 Current')
plt.plot(df['servo2_current'], label='Servo 2 Current')
plt.plot(df['servo3_current'], label='Servo 3 Current')
plt.plot(df['servo4_current'], label='Servo 4 Current')
plt.plot(df['servo5_current'], label='Servo 5 Current')

# Adding labels and title
plt.xlabel('Index')
plt.ylabel('Current')
plt.title('Servo Currents')
plt.legend()

# Show the plot
plt.show()



# Calculate the cumulative sum for each servo_current column
for i in range(1, 6):
    servo_col = f'servo{i}_current'
    df[f'{servo_col}_cumsum'] = df[servo_col].cumsum()

# Plotting the cumulative sum for each servo's current
plt.figure(figsize=(10, 6))

for i in range(1, 6):
    servo_col = f'servo{i}_current_cumsum'
    plt.plot(df[servo_col], label=f'Servo {i} Current Cumulative Sum')

# Adding labels and title
plt.xlabel('Index')
plt.ylabel('Cumulative Current')
plt.title('Cumulative Sum of Servo Currents')
plt.legend()

# Show the plot
plt.show()

print(df.head())