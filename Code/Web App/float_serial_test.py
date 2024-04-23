import serial
import struct
import time

# Adjust these variables to match your setup
SERIAL_PORT = 'COM5'  # Change this to your serial port
BAUD_RATE = 9600  # Baud rate used in the Arduino sketch
FLOAT_VALUE = 123.456  # The float you want to send

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Give the Arduino time to reset (if it does on serial connection)
time.sleep(2)

# Send the float value to the Arduino
float_bytes = struct.pack('f', FLOAT_VALUE)  # Convert the float to bytes
ser.write(float_bytes)

# Read the response from the Arduino
response_bytes = ser.read(4)  # Read 4 bytes (size of a float)
received_float = struct.unpack('<f', response_bytes)[0]  # Unpack the bytes to a float

print("Sent float:", FLOAT_VALUE)
print("Received float:", received_float)