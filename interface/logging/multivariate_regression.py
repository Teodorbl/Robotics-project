# FILE: MULTIVATIATE_REGRESSION.PY

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score

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
    # A new group is assigned when the current command differs from the previous one
    commands[f'servo{i}_group'] = (commands[servo_col] != commands[servo_col].shift(1)).cumsum()

# Merge commands with feedbacks based on command_id
feedbacks = feedbacks.merge(
    commands[['command_id', 'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
              'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group']],
    on='command_id',
    how='left'
)

# Extract Uno_Current and Nano_Servo5_Current from feedbacks
# Assuming all 5 currents are sent together by the same Arduino Nano
# Example content: "Nano_Current: 530,535,528,529,532"
feedbacks[['servo1_current', 'servo2_current', 'servo3_current', 'servo4_current', 'servo5_current']] = feedbacks['content'].str.extract(
    r'Nano_Current:\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)'
)

# Convert current columns to numeric, coercing errors to NaN
for i in range(1, 6):
    feedbacks[f'servo{i}_current'] = pd.to_numeric(feedbacks[f'servo{i}_current'], errors='coerce')

# Reshape feedbacks to long format for easier processing
feedbacks_long = feedbacks.melt(
    id_vars=['timestamp', 'event_type', 'content', 'command_id',
             'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
             'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group'],
    value_vars=['servo1_current', 'servo2_current', 'servo3_current', 'servo4_current', 'servo5_current'],
    var_name='servo',
    value_name='current'
)

# Drop NaN current readings
feedbacks_long = feedbacks_long.dropna(subset=['current'])

# Extract servo number from 'servo' column
feedbacks_long['servo_num'] = feedbacks_long['servo'].str.extract(r'servo(\d+)_current').astype(int)

# Assign group_id based on servo_num and corresponding servo_group
feedbacks_long['group_id'] = feedbacks_long.apply(
    lambda row: row[f'servo{row["servo_num"]}_group'],
    axis=1
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

# Aggregate current readings per servo and group_id using Mean
aggregated_current = feedbacks_long.groupby(['servo_num', 'group_id']).agg({
    'command_angle': 'first',
    'current': 'mean'
}).reset_index()

# Map analog readings to Amps
servo_current_offsets = [524, 531, 530, 529, 531]

# Define a function to convert analog readings to Amps
def analog_to_amps(row):
    servo_index = int(row['servo_num'] - 1)  # Adjusting for zero-based index
    analog_read = row['current']
    offset = servo_current_offsets[servo_index]
    amps = 5 * (analog_read - offset) / (1023 * 0.185)
    return amps

# Apply the mapping to create a new column 'current_amps'
aggregated_current['current_amps'] = aggregated_current.apply(analog_to_amps, axis=1)

# Extract Feedbacks (Assuming similar processing as current readings)
# For demonstration, let's assume feedbacks are already aggregated similarly
# If not, similar steps should be followed to aggregate feedbacks
# Here, we'll perform a mock aggregation for feedbacks

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
feedbacks_fb_long = feedbacks.melt(
    id_vars=['timestamp', 'event_type', 'content', 'command_id',
             'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
             'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group'],
    value_vars=['servo1_fb', 'servo2_fb', 'servo3_fb', 'servo4_fb', 'servo5_fb'],
    var_name='servo',
    value_name='feedback'
)

# Drop NaN feedbacks
feedbacks_fb_long = feedbacks_fb_long.dropna(subset=['feedback'])

# Extract servo number from 'servo' column
feedbacks_fb_long['servo_num'] = feedbacks_fb_long['servo'].str.extract(r'servo(\d)_fb').astype(int)

# Assign group_id based on servo_num and corresponding servo_group
feedbacks_fb_long['group_id'] = feedbacks_fb_long.apply(
    lambda row: row[f'servo{row["servo_num"]}_group'],
    axis=1
)

# Merge with commands to get command angles
feedbacks_fb_long = feedbacks_fb_long.merge(
    commands[['command_id', 'servo1_cmd', 'servo2_cmd', 'servo3_cmd', 'servo4_cmd', 'servo5_cmd',
              'servo1_group', 'servo2_group', 'servo3_group', 'servo4_group', 'servo5_group']],
    on='command_id',
    how='left',
    suffixes=('', '_cmd')
)

# Select relevant command angle based on servo_num
feedbacks_fb_long['command_angle'] = feedbacks_fb_long.apply(get_command_angle, axis=1)

# Aggregate feedback readings per servo and group_id using Mean
aggregated_feedback = feedbacks_fb_long.groupby(['servo_num', 'group_id']).agg({
    'command_angle': 'first',
    'feedback': 'mean'
}).reset_index()

# Merge aggregated_current with aggregated_feedback
merged_data = pd.merge(
    aggregated_current,
    aggregated_feedback,
    on=['servo_num', 'group_id', 'command_angle'],
    how='inner'
)

# Display the merged DataFrame (optional)
print("Merged Aggregated Data:")
print(merged_data.head())

# Multivariate Linear Regression
# Dependent Variable: command_angle
# Independent Variables: feedback, current_amps

# Prepare the data
X = merged_data[['feedback', 'current_amps']]
y = merged_data['command_angle']

# Initialize the model
model = LinearRegression()

# Fit the model
model.fit(X, y)

# Predict using the model
y_pred = model.predict(X)

# Calculate R-squared
r2 = r2_score(y, y_pred)

# Display regression coefficients
print("\nMultivariate Linear Regression Results:")
print(f"Intercept: {model.intercept_:.4f}")
print(f"Coefficient for Feedback: {model.coef_[0]:.4f}")
print(f"Coefficient for Current (Amps): {model.coef_[1]:.4f}")
print(f"R-squared: {r2:.4f}")

# Plotting the Residuals
plt.figure(figsize=(10, 6))
sns.scatterplot(x=y, y=y - y_pred, alpha=0.6)
plt.axhline(0, color='red', linestyle='--')
plt.title('Residuals Plot')
plt.xlabel('Actual Command Angle (Degrees)')
plt.ylabel('Residuals')
plt.tight_layout()
plt.show()

# Optional: Display a summary table
regression_results = {
    'Intercept': [model.intercept_],
    'Feedback Coef': [model.coef_[0]],
    'Current_Amp Coef': [model.coef_[1]],
    'R-squared': [r2]
}

regression_df = pd.DataFrame(regression_results)
print("\nRegression Summary:")
print(regression_df)