# FILE:FULL_INTERFACE.PY

import sys
import re

# --------------------------------------------
# Import and create constants
# --------------------------------------------

# Function to parse ServoConfig.h and extract constants
def parse_servo_config(config_path):
    constants = {}
    with open(config_path, 'r') as file:
        for line in file:
            # Match lines like #define CONSTANT_NAME value with optional leading whitespace
            define_match = re.match(r'\s*#define\s+(\w+)\s+(\d+)', line)
            if define_match:
                key, value = define_match.groups()
                constants[key] = int(value)
            # Match lines like const type NAME[NUM] = {values}; with optional leading whitespace and pointer types
            array_match = re.match(r'\s*const\s+([\w\*]+)\s+(\w+)\s*\[\s*\w+\s*\]\s*=\s*\{([^}]+)\};', line)
            if array_match:
                type_, key, values = array_match.groups()
                if type_.lower() == 'bool':
                    constants[key] = [v.strip().lower() == 'true' for v in values.split(',')]
                elif type_.lower() == 'char*':
                    constants[key] = [v.strip().strip('"') for v in values.split(',')]
                else:
                    constants[key] = [int(v.strip()) for v in values.split(',')]
            
            # ...existing code...
            
            # Add handling for scalar const declarations
            scalar_match = re.match(r'\s*const\s+(\w+)\s+(\w+)\s*=\s*([^\s;]+)\s*;', line)
            if scalar_match:
                type_, key, value = scalar_match.groups()
                if type_.lower() in ['int', 'float']:
                    constants[key] = int(value) if type_.lower() == 'int' else float(value)
                elif type_.lower() == 'bool':
                    constants[key] = value.strip().lower() == 'true'
                elif type_.lower() in ['char*', 'const char*']:
                    constants[key] = value.strip().strip('"')
    return constants

# Parse ServoConfig.h
servo_config_path = 'include/ServoConfig.h'
try:
    config = parse_servo_config(servo_config_path)
except Exception as e:
    print(f"Error parsing servo configuration: {e}")
    sys.exit(1)

# Extract constants and raise errors if any are missing
required_keys = [
    'NUM_SERVOS', 'SERVO_NAMES', 'SERVO_MIN_ANGLES', 'SERVO_MAX_ANGLES',
    'SERVO_INVERT_MASK', 'SERVO_DEFAULT_ANGLES', 'SERVO_MIN_DEGREE',
    'SERVO_MAX_DEGREE', 'SERVO_MIN_PULSE_WIDTH', 'SERVO_MAX_PULSE_WIDTH',
    'I2C_ADDRESS_NANO', 'I2C_ADDRESS_PWMDRV'
]

for key in required_keys:
    if key not in config:
        raise KeyError(f"Missing required configuration key: {key}")

NUM_SERVOS = config['NUM_SERVOS']
SERVO_NAMES = config['SERVO_NAMES']
SERVO_MIN_ANGLES = config['SERVO_MIN_ANGLES']
SERVO_MAX_ANGLES = config['SERVO_MAX_ANGLES']
SERVO_INVERT_MASK = config['SERVO_INVERT_MASK']
SERVO_DEFAULT_ANGLES = config['SERVO_DEFAULT_ANGLES']
SERVO_MIN_DEGREE = config['SERVO_MIN_DEGREE']
SERVO_MAX_DEGREE = config['SERVO_MAX_DEGREE']
SERVO_MIN_PULSE_WIDTH = config['SERVO_MIN_PULSE_WIDTH']
SERVO_MAX_PULSE_WIDTH = config['SERVO_MAX_PULSE_WIDTH']
I2C_ADDRESS_NANO = config['I2C_ADDRESS_NANO']
I2C_ADDRESS_PWMDRV = config['I2C_ADDRESS_PWMDRV']

# Configure the serial ports for both Nano and Uno
SERIAL_PORT_UNO = '/dev/cu.usbmodem11101'
SERIAL_PORT_NANO = '/dev/cu.usbserial-A105A9FZ'

# Initialize baud rates after removing BAUD_RATE
BAUD_RATE_UNO = 115200
BAUD_RATE_NANO = 57600
