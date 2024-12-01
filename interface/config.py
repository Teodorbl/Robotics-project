import re

def load_config(config_path='/Users/teodorlindell/Repos/RoboticArmFreeRTOS/include/ServoConfig.h'):
    """
    Parse the ServoConfig.h file and extract configuration constants.
    
    Args:
        config_path (str): Path to the ServoConfig.h file.
    
    Returns:
        dict: A dictionary containing all extracted configuration constants.
    """
    constants = {}
    with open(config_path, 'r') as file:
        for line in file:
            # Match lines like #define CONSTANT_NAME value with optional leading whitespace
            define_match = re.match(r'\s*#define\s+(\w+)\s+(\d+)', line)
            if define_match:
                key, value = define_match.groups()
                constants[key] = int(value)
            
            # Match lines like const type NAME[NUM] = {values};
            array_match = re.match(r'\s*const\s+([\w\*]+)\s+(\w+)\s*\[\s*\w+\s*\]\s*=\s*\{([^}]+)\};', line)
            if array_match:
                type_, key, values = array_match.groups()
                if type_.lower() == 'bool':
                    constants[key] = [v.strip().lower() == 'true' for v in values.split(',')]
                elif type_.lower() == 'char*':
                    constants[key] = [v.strip().strip('"') for v in values.split(',')]
                else:
                    constants[key] = [int(v.strip()) for v in values.split(',')]
            
            # Match lines like const type NAME = value;
            scalar_match = re.match(r'\s*const\s+(\w+)\s+(\w+)\s*=\s*([^\s;]+)\s*;', line)
            if scalar_match:
                type_, key, value = scalar_match.groups()
                if type_.lower() in ['int', 'float']:
                    constants[key] = int(value) if type_.lower() == 'int' else float(value)
                elif type_.lower() == 'bool':
                    constants[key] = value.strip().lower() == 'true'
                elif type_.lower() in ['char*', 'const char*']:
                    constants[key] = value.strip().strip('"')
                    
    # Interface related configs
    interface_configs = {
        'SERIAL_PORT_UNO':  '/dev/cu.usbmodem11101',
        'SERIAL_PORT_NANO':  '/dev/cu.usbserial-A105A9FZ'
    }
    
    constants.update(interface_configs)
    
    return constants
