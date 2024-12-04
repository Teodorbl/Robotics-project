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

# Assign group_id for each servo based on command changes
for i in range(1, 6):
    servo_col = f'servo{i}_cmd'
    commands[f'servo{i}_group'] = (commands[servo_col] != commands[servo_col].shift(1)).cumsum()

# Merge commands with feedbacks based on command_id
feedbacks = feedbacks.merge(
    commands[['command_id', 'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
              'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group']],
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

# Reshape feedbacks to long format for easier processing
feedbacks_long = feedbacks.melt(
    id_vars=['timestamp', 'event_type', 'content', 'command_id',
             'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
             'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group'],
    value_vars=['servo1_fb', 'servo2_fb', 'servo3_fb', 'servo4_fb', 'servo5_fb'],
    var_name='servo',
    value_name='feedback'
)

# Drop NaN feedbacks
feedbacks_long = feedbacks_long.dropna(subset=['feedback'])

# Extract servo number from 'servo' column
feedbacks_long['servo_num'] = feedbacks_long['servo'].str.extract(r'servo(\d)_fb').astype(int)

# Assign group_id based on servo_num and corresponding servo_group
feedbacks_long['group_id'] = feedbacks_long.apply(
    lambda row: row[f'servo{row["servo_num"]}_group'], axis=1
)

# Merge with commands to get command angles
feedbacks_long = feedbacks_long.merge(
    commands[['command_id', 'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
              'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group']],
    on='command_id',
    how='left',
    suffixes=('', '_cmd')
)

# Select relevant command angle based on servo_num
def get_command_angle(row):
    return row[f'servo{row["servo_num"]}_cmd']

feedbacks_long['command_angle'] = feedbacks_long.apply(get_command_angle, axis=1)

# Aggregate feedbacks per servo and group_id
aggregated_feedback = feedbacks_long.groupby(['servo_num', 'group_id']).agg({
    'command_angle': 'first',
    'feedback': 'mean'
}).reset_index()

# Display the aggregated DataFrame (optional)
print(aggregated_feedback.head())

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
    servo_feedback = aggregated_feedback[aggregated_feedback['servo_num'] == i]['feedback']
    sns.histplot(servo_feedback, label=f'Servo {i} Feedback', kde=False, bins=20, alpha=0.5)
plt.title('Histogram of Servo Feedbacks')
plt.xlabel('Servo Angle (Degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.tight_layout()
plt.show()

# Correlation Analysis: Command vs. Feedback for Each Servo

for i in range(1, 6):
    servo_data = aggregated_feedback[aggregated_feedback['servo_num'] == i]
    plt.figure(figsize=(8, 6))
    sns.scatterplot(
        x=servo_data['command_angle'],
        y=servo_data['feedback'],
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
    servo_data = aggregated_feedback[aggregated_feedback['servo_num'] == i]
    correlation = servo_data['command_angle'].corr(servo_data['feedback'])
    correlation_data[f'servo{i}'] = correlation

correlation_df = pd.DataFrame.from_dict(correlation_data, orient='index', columns=['Correlation'])
print("\nCorrelation Coefficients between Commands and Feedbacks:")
print(correlation_df)