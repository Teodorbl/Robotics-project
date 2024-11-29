import serial
import sys
import time
import threading
import select

# Configuration Constants
SERIAL_PORT = '/dev/tty.usbmodem11101'  # Adjust as needed
BAUD_RATE = 9600
RETRY_DELAY = 5  # seconds


class SerialReader(threading.Thread):
    """
    Thread responsible for continuously reading from the serial port.
    """

    def __init__(self, ser, stop_event):
        super().__init__(daemon=True)
        self.ser = ser
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            try:
                if self.ser.in_waiting:
                    # Read a line from the serial port
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        print(f"UNO: {line}")
                else:
                    time.sleep(0.1)  # Prevents busy-waiting
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.stop_event.set()
            except Exception as e:
                print(f"Unexpected error: {e}")
                self.stop_event.set()


def main():
    # Event to signal threads to stop
    stop_event = threading.Event()

    # Establish serial connection
    while not stop_event.is_set():
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            break
        except serial.SerialException as e:
            print(f"Error opening serial port {SERIAL_PORT}: {e}")
            print(f"Retrying in {RETRY_DELAY} seconds...")
            time.sleep(RETRY_DELAY)
    else:
        return

    # Start the serial reader thread
    reader = SerialReader(ser, stop_event)
    reader.start()

    try:
        print("Enter commands in the format 'servoName angle' or 'servoIndex angle'. Type 'exit' to quit.")
        while not stop_event.is_set():
            # Check if user input is available
            if select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip()
                if user_input.lower() in ['exit', 'quit']:
                    print("Exiting program.")
                    stop_event.set()
                elif user_input:
                    # Send the user input to the serial port
                    ser.write((user_input + '\n').encode('utf-8'))
                else:
                    # Empty input, do nothing
                    pass
            else:
                time.sleep(0.1)  # Prevents busy-waiting
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Exiting...")
        stop_event.set()
    finally:
        # Ensure threads are stopped and serial port is closed
        stop_event.set()
        reader.join()
        ser.close()
        print("Program terminated.")

if __name__ == "__main__":
    main()