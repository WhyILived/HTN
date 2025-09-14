import serial
import time

class SerialCommunication:
    def __init__(self, port="COM15", baud=115200, timeout=1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)  # give ESP32 time to reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"Connected to {self.port} at {self.baud} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to serial port {self.port}: {e}")
            self.ser = None

    def safe_write(self, data: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                self.ser.flush()
            except serial.SerialException as e:
                print(f"Write error: {e}")
                self.connect()
        else:
            print("Serial port not open. Reconnecting...")
            self.connect()

    def string(self, s: str):
        self.safe_write(b"String: " + s.encode() + b"\n")

    def direction(self, d: str):
        self.safe_write(b"Braille: " + d.encode() + b"\n")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")