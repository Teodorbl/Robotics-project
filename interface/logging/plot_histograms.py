# FILE: PLOT_HISTOGRAMS.PY

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Read the log file
log_file = "command_feedback_log.csv"
df = pd.read_csv(log_file)

# Convert timestamp to datetime for better readability
df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')

# Separate commands and feedbacks
commands = df[df['event_type'] == 'command_sent']
feedbacks = df[df['event_type'] == 'feedback_received']

# Extract servo angles from commands
# Assuming command format: "<090,035,160,090,120>"
commands[['servo1', 'servo2', 'servo3', 'servo4', 'servo5']] = commands['content'].str.extract(
    r'<(\d{3}),(\d{3}),(\d{3}),(\d{3}),(\d{3})>'
)

# Convert extracted servo angles to integers
for i in range(1, 6):
    commands[f'servo{i}'] = commands[f'servo{i}'].astype(int)

# Example: Histogram of Servo Commands
plt.figure(figsize=(12, 6))
for i in range(1, 6):  # Servos 1 to 5
    servo_angle = commands[f'servo{i}']
    sns.histplot(servo_angle, label=f'Servo {i}', kde=False, bins=20, alpha=0.5)
plt.title('Histogram of Servo Commands')
plt.xlabel('Servo Angle (Degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.tight_layout()
plt.show()

# Extract and plot Servo Feedbacks
plt.figure(figsize=(12, 6))
for i in range(5):  # Assuming 5 servos
    if i < 4:
        # From Uno Feedback (positions), assuming format "Uno_FB: 342,218,215,339"
        # Extract all four servo feedbacks from Uno
        uno_feedback = feedbacks['content'].str.extract(
            r'Uno_FB: (\d+),(\d+),(\d+),(\d+)'
        )
        if not uno_feedback.empty:
            servo_fb = uno_feedback[i].astype(float)
            sns.histplot(servo_fb, label=f'Uno Servo {i+1} Feedback', kde=False, bins=20, alpha=0.5)
    else:
        # Servo 5 from Nano Feedback, format "Nano_Servo5_FB: 402"
        nano_servo5 = feedbacks.loc[
            feedbacks['content'].str.startswith('Nano_Servo5_FB:'), 'content'
        ].str.extract(r'Nano_Servo5_FB: (\d+)')[0].dropna()

        if not nano_servo5.empty:
            servo5_fb = nano_servo5.astype(int)
            sns.histplot(servo5_fb, label='Nano Servo 5 Feedback', kde=False, bins=20, alpha=0.5)

plt.title('Histogram of Servo Feedbacks')
plt.xlabel('Servo Angle (Degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.tight_layout()
plt.show()

# Example: Correlation between Commands and Feedbacks
# This assumes that commands and feedbacks are in order

# Merge commands and feedbacks based on nearest timestamp within a tolerance
merged_df = pd.merge_asof(
    commands.sort_values('timestamp'),
    feedbacks.sort_values('timestamp'),
    on='timestamp',
    direction='forward',
    tolerance=0.1  # Adjust tolerance as needed (seconds)
)

# Extract Servo1 Command and Servo1 Feedback for correlation
# First, ensure the 'Uno_FB' is present in 'content_y'
merged_df = merged_df.dropna(subset=['content_y'])

# Extract Servo1 Command
merged_df['servo1_command'] = merged_df['content_x'].str.extract(r'<(\d{3}),')[0].astype(int)

# Extract Servo1 Feedback from Uno
merged_df[['servo1_feedback', '_', '_', '_']] = merged_df['content_y'].str.extract(
    r'Uno_FB: (\d+),(\d+),(\d+),(\d+)'
)

merged_df['servo1_feedback'] = merged_df['servo1_feedback'].astype(float)

# Plot Servo1 Command vs Servo1 Feedback
plt.figure(figsize=(8, 6))
plt.scatter(
    merged_df['servo1_command'],
    merged_df['servo1_feedback'],
    alpha=0.5
)
plt.title('Servo 1 Command vs Servo 1 Feedback')
plt.xlabel('Servo 1 Command (Degrees)')
plt.ylabel('Servo 1 Feedback (Degrees)')
plt.grid(True)
plt.tight_layout()
plt.show()