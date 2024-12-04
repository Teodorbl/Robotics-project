# FILE: PLOT_HISTOGRAMS.PY

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Read the log file
log_file = "command_feedback_log.csv"
df = pd.read_csv(log_file)

# Convert timestamp to datetime for better readability
df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')

# Sort the DataFrame by timestamp to ensure chronological order
df = df.sort_values('timestamp').reset_index(drop=True)

# Assign a unique command_id to each 'command_sent' event
df['command_id'] = df['event_type'].eq('command_sent').cumsum()

# Forward-fill the command_id for 'feedback_received' events
df['command_id'] = df['command_id'].where(df['event_type'] == 'command_sent').ffill()

# Separate commands and feedbacks
commands = df[df['event_type'] == 'command_sent'].copy()
feedbacks = df[df['event_type'] == 'feedback_received'].copy()

# Extract servo angles from commands
# Assuming command format: "<090,035,160,090,120>"
commands[['servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd']] = commands['content'].str.extract(
    r'<(\d{3}),(\d{3}),(\d{3}),(\d{3}),(\d{3})>'
)

# Convert extracted servo angles to integers
for i in range(1, 6):
    commands[f'servo{i}_cmd'] = commands[f'servo{i}_cmd'].astype(int)

# Merge commands with feedbacks based on command_id
feedbacks = feedbacks.merge(
    commands[['command_id', 'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd']],
    on='command_id',
    how='left'
)

# Extract Uno_FB and Nano_Servo5_FB from feedbacks
# Extract Uno_FB: servo1-4 feedback positions
feedbacks[['servo1_fb', 'servo2_fb', 'servo3_fb', 'servo4_fb']] = feedbacks['content'].str.extract(
    r'Uno_FB: (\d+),(\d+),(\d+),(\d+)'
)

# Extract Nano_Servo5_FB: servo5 feedback
feedbacks['servo5_fb'] = feedbacks['content'].str.extract(
    r'Nano_Servo5_FB: (\d+)'
)

# Convert feedback columns to numeric, coercing errors to NaN
for i in range(1, 6):
    feedbacks[f'servo{i}_fb'] = pd.to_numeric(feedbacks[f'servo{i}_fb'], errors='coerce')

# Aggregate feedbacks per command_id by calculating the mean feedback for each servo
aggregated_feedback = feedbacks.groupby('command_id').agg({
    'servo1_fb': 'mean',
    'servo2_fb': 'mean',
    'servo3_fb': 'mean',
    'servo4_fb': 'mean',
    'servo5_fb': 'mean'
}).reset_index()

# Merge aggregated feedbacks with commands
merged_df = commands.merge(aggregated_feedback, on='command_id', how='left')

# Drop any commands without feedbacks (optional)
merged_df = merged_df.dropna(subset=['servo1_fb', 'servo2_fb', 'servo3_fb', 'servo4_fb', 'servo5_fb'])

# Display the merged DataFrame (optional)
print(merged_df.head())

# Plotting Histograms of Servo Commands
plt.figure(figsize=(12, 6))
for i in range(1, 6):  # Servos 1 to 5
    servo_angle = commands[f'servo{i}_cmd']
    sns.histplot(servo_angle, label=f'Servo {i}', kde=False, bins=20, alpha=0.5)
plt.title('Histogram of Servo Commands')
plt.xlabel('Servo Angle (Degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.tight_layout()
plt.show()

# Plotting Histograms of Servo Feedbacks
plt.figure(figsize=(12, 6))
for i in range(1, 6):  # Servos 1 to 5
    servo_fb = merged_df[f'servo{i}_fb']
    sns.histplot(servo_fb, label=f'Servo {i} Feedback', kde=False, bins=20, alpha=0.5)
plt.title('Histogram of Servo Feedbacks')
plt.xlabel('Servo Angle (Degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.tight_layout()
plt.show()

# Correlation Analysis: Command vs. Feedback for Each Servo

for i in range(1, 6):
    plt.figure(figsize=(8, 6))
    sns.scatterplot(
        x=merged_df[f'servo{i}_cmd'],
        y=merged_df[f'servo{i}_fb'],
        alpha=0.6
    )
    plt.title(f'Servo {i} Command vs Servo {i} Feedback')
    plt.xlabel(f'Servo {i} Command (Degrees)')
    plt.ylabel(f'Servo {i} Feedback (Degrees)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Optional: Calculate and Display Correlation Coefficients
correlation_data = {}
for i in range(1, 6):
    correlation = merged_df[f'servo{i}_cmd'].corr(merged_df[f'servo{i}_fb'])
    correlation_data[f'servo{i}'] = correlation

correlation_df = pd.DataFrame.from_dict(correlation_data, orient='index', columns=['Correlation'])
print("\nCorrelation Coefficients between Commands and Feedbacks:")
print(correlation_df)