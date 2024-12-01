from config import load_config
from serial_handler import SerialHandler
from gui import setup_gui

def main():
    # Load configuration
    config = load_config()
    
    # Initialize serial communication
    serial_handler = SerialHandler(config)
    
    # Setup and launch GUI
    setup_gui(serial_handler, config)

if __name__ == "__main__":
    main()
