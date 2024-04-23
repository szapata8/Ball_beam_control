import serial
import serial.tools.list_ports
import struct
import time


HWID = "10C4:EA60" # ESP32 serial hwid
MICROCONTROLLER_BAUD_RATE = 115200
ANGLE_CALIBRATION_OFFSET = 0
DISTANCE_CALIBRATION_OFFSET = 0 # mm

def findUsbPort(hwid):
    """ Find the COM port name based of off the hwid parameter in each COM port connection

    Args:
        hwid (str): desired hardware identifier

    Returns:
        str or None: COM port name of the device that has the provided hardware identifier
    """
    ports = list(serial.tools.list_ports.comports())
    # print(f"hwid.upper(): {hwid.upper()}")
    for p in ports:
        # print(f"p.hwid: {p.hwid}")
        if hwid.upper() in p.hwid:
            # print("Found '{}' at '{}'".format(p.hwid, p.device))
            return p.device
        # else:
            # print("not correct hwid. ")
    return None 

def send_float(serial_handle, float_value):
    data = struct.pack('f', float_value)
    serial_handle.write(data)

class SeeSawControl:
    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        self.serial_handle = serial.Serial(baudrate=MICROCONTROLLER_BAUD_RATE, timeout=1)
        self.serial_handle.hwid = HWID
        self.serial_handle.port = None
        self.angle = 0
        self.target_angle = 0
        self.ball_position = 0
        self.real_zero = None
        self.kp = 0
        self.ki = 0
        self.kd = 0

    def connect(self):
        self.serial_handle.port = findUsbPort(self.serial_handle.hwid)
        if self.serial_handle.port is None:
            raise ValueError('Microcontroller not found')
        if self.serial_handle.is_open:
            self.serial_handle.close()
        self.serial_handle.open()
        print("Waiting to flush buffers...")
        self.serial_handle.reset_input_buffer()
        self.serial_handle.reset_output_buffer()
        self.serial_handle.flush()
        time.sleep(5)
        print("Done flushing buffers!")

    def send(self, command):
        encoded_command = bytes(command)
        self.serial_handle.write(encoded_command)

    # ============ MANUAL CONTROL
    def change_to_manual_control_modality(self):
        self.send(b"@")
        self.send(b"M")
        echo = self.serial_handle.read(1)
        print(f"modality: {echo}")

    def step_ccw(self):
        print(f"SeeSawControl.step_ccw()")
        if self.serial_handle.port is not None:
            self.send(b"s")

    def step_cw(self):
        print(f"SeeSawControl.step_cw()")
        if self.serial_handle.port is not None:
            self.send(b"S")

    def microstep_ccw(self):
        print(f"SeeSawControl.microstep_ccw()")
        if self.serial_handle.port is not None:
            self.send(b"u")

    def microstep_cw(self):
        print(f"SeeSawControl.microstep_cw()")
        if self.serial_handle.port is not None:
            self.send(b"U")

    # ============ SYSTEM STATES
    def request_angle(self):
        # print(f"SeeSawControl.request_angle()")
        if self.serial_handle.port is not None:
            self.send(b"r")
            while self.serial_handle.in_waiting < 4:
                pass
            float_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a float
            self.angle = struct.unpack('f', float_bytes)[0]
            self.angle -= ANGLE_CALIBRATION_OFFSET
            self.angle = round(self.angle, 1)
            if (self.debug_mode):
                print(f"self.angle: {self.angle}")
    
    def get_angle(self):
        try:
            self.request_angle()
        except:
            pass
        return self.angle
    
    def zero_reference_angle(self):
        print(f"SeeSawControl.zero_reference_angle()")
        if self.serial_handle.port is not None:
            self.send(b"z")
            while self.serial_handle.in_waiting < 4:
                pass
            float_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a float
            self.real_zero = struct.unpack('f', float_bytes)[0]
            if (self.debug_mode):
                print(f"self.real_zero: {self.real_zero}")

    def request_ball_position(self):
        # print(f"SeeSawControl.request_ball_position()")
        if self.serial_handle.port is not None:
            self.send(b"d")
            while self.serial_handle.in_waiting < 4:
                pass
            position_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a double
            self.ball_position = struct.unpack('f', position_bytes)[0]
            self.ball_position -= DISTANCE_CALIBRATION_OFFSET
            if (self.debug_mode):
                print(f"self.ball_position: {self.ball_position}")
    
    def get_ball_position(self):
        # self.request_ball_position()
        self.ball_position = 25
        return self.ball_position

    # ============ PID CONTROL
    def change_to_pid_control_modality(self):
        self.send(b"@")
        self.send(b"P")
        echo = self.serial_handle.read(1)
        if (self.debug_mode):
            print(f"modality echo: {echo}")

    def set_target_angle(self, target_angle):
        # print(f"SeeSawControl.set_kp(), kp_value: {kp_value}. type: {type(kp_value)}")
        if self.serial_handle.port is not None:
            # Send command indicator
            self.send(b"a")

            # Send command value
            send_float(self.serial_handle, target_angle)

            # Listen to echo
            while self.serial_handle.in_waiting < 4:
                pass
            float_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a float
            self.target_angle = struct.unpack('<f', float_bytes)[0]
            if (self.debug_mode):
                print(f"self.target_angle: {self.target_angle}")

    def get_kp(self):
        pass

    def get_ki(self):
        pass
    
    def get_kd(self):
        pass

    def set_kp(self, kp_value):
        # print(f"SeeSawControl.set_kp(), kp_value: {kp_value}. type: {type(kp_value)}")
        if self.serial_handle.port is not None:
            # Send command indicator
            self.send(b"p")

            # Send command value
            send_float(self.serial_handle, kp_value)

            # Listen to echo
            while self.serial_handle.in_waiting < 4:
                pass
            float_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a float
            self.kp = struct.unpack('<f', float_bytes)[0]
            if (self.debug_mode):
                print(f"self.kp: {self.kp}")

    def set_ki(self, ki_value):
        # print(f"SeeSawControl.set_ki(), kp_value: {ki_value}. type: {type(ki_value)}")
        if self.serial_handle.port is not None:
            # Send command indicator
            self.send(b"i")

            # Send command value
            send_float(self.serial_handle, ki_value)

            # Listen to echo
            while self.serial_handle.in_waiting < 4:
                pass
            float_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a float
            self.ki = struct.unpack('<f', float_bytes)[0]
            if (self.debug_mode):
                print(f"self.ki: {self.ki}")

    def set_kd(self, kd_value):
        # print(f"SeeSawControl.set_kd(), kd_value: {kd_value}. type: {type(kd_value)}")
        if self.serial_handle.port is not None:
            # Send command indicator
            self.send(b"d")

            # Send command value
            send_float(self.serial_handle, kd_value)

            # Listen to echo
            while self.serial_handle.in_waiting < 4:
                pass
            float_bytes = self.serial_handle.read(4)
        
            # Convert the bytes to a float
            self.kd = struct.unpack('<f', float_bytes)[0]
            if (self.debug_mode):
                print(f"self.kd: {self.kd}")

def testing():
    seesaw = SeeSawControl(debug_mode=True)
    seesaw.connect()

    # while True:
        # seesaw.get_angle()
        # time.sleep(0.1)
    # time.sleep(5)
    # change modality
    seesaw.change_to_pid_control_modality()
    # time.sleep(1)
    seesaw.set_kp(0.5)
    seesaw.set_ki(0.4)
    seesaw.set_kd(0.3)
    seesaw.change_to_manual_control_modality()
    
    while True:
        seesaw.get_angle()
        time.sleep(0.1)

if __name__ == "__main__":
    testing()
