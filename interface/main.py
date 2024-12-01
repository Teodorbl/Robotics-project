import atexit
import signal
import sys
from pyqtgraph.Qt.QtWidgets import QApplication

import backend.configs as configs
from backend.serial_api import SerialAPI
from frontend.gui import GUI

def main():
    app = QApplication(sys.argv)  # Create QApplication instance
    serial_api = SerialAPI(configs)
    gui = GUI(configs, serial_api)
    serial_api.gui = gui
    
    # On exit:
    
    atexit.register(lambda: serial_api.close())

    # Signal handler for graceful exit
    def signal_handler(sig, frame):
        gui.append_output("\nKeyboardInterrupt detected. Exiting gracefully...")
        gui.close()

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        sys.exit(app.exec_())  # Start event loop using app
    except SystemExit:
        pass


if __name__ == '__main__':
    main()