import atexit
import signal
import sys


import backend.configs as configs
from backend.serial_api import SerialAPI
from frontend.gui import GUI

def main():
    serial_api = SerialAPI(configs)
    gui = GUI(configs, serial_api)
    serial_api.gui = gui
    
    # On exit:
    
    atexit.register(lambda: serial_api.close())

    # Modify the signal_handler to call reset_servos before exiting
    def signal_handler(sig, frame):
        gui.append_output("\nKeyboardInterrupt detected. Exiting gracefully...")
        gui.close()

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print("Opening app")
        sys.exit(gui.window.app.exec_())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()