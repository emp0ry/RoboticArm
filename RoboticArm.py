from math import radians, degrees, atan2, acos, sqrt, cos, sin
from serial import Serial
from time import sleep

class RAConfig:
    def __init__(self, serial_port: str):
        # Serial port and baund rate of SSC-32U
        self.serial_port = serial_port
        self.baud_rate = 115200

        # Servo motors pins
        self.base_pin = 0
        self.shoulder_pin = 1
        self.elbow_pin = 16
        self.effector_pin = 17
        self.gripper_pin = 18

        # Force sensor pin
        self.force_sensor_pin = 'A'

        # Min/Max degrees and microseconds for make degree to microsecond
        self.min_degree = 0
        self.max_degree = 180
        self.min_ms = 500
        self.max_ms = 2400
        self.gripper_min_ms = 500
        self.gripper_max_ms = 1300

        # Default arguments for move() and move_fromto() functions
        self.step = 0.1
        self.speed = 700
        self.effector_horizontal_degree = 90

        # Defualt argument for grap() function
        self.max_strength = 20

        # Inverse kinematics calculation lengths of the arm links in cm
        self.link1 = 12.6
        self.link2 = 14.38
        self.link3 = 16.43
        self.link4 = 13.7

        self.ser = Serial(self.serial_port, self.baud_rate, timeout=0.1)

    def inverse_kinematics(self, x: float, y: float, z: float, thetaAT: float=90):
        # Distance between the base motor and the end effector in the x-y plane
        r = sqrt(x**2 + y**2)

        rW = r - self.link4 * sin(radians(thetaAT))
        z0 = z - self.link1 + self.link4 * cos(radians(thetaAT))
        betta = degrees(atan2(z0, rW))

        # Distance between the second joint and the end effector
        r0 = sqrt(z0**2 + rW**2)

        alpha = degrees(acos((r0**2 + self.link2**2 - self.link3**2) / (2 * r0 * self.link2)))

        # Angles for the motors
        theta1 = degrees(atan2(y, x))
        theta2 = alpha + betta
        theta3 = degrees(acos((self.link2**2 + self.link3**2 - r0**2) / (2 * self.link2 * self.link3)))
        theta4 = 270 + thetaAT - theta2 - theta3

        return theta1, theta2, theta3, theta4

    def move(self, x: float, y: float, z: float, speed: int=700, effector_horizontal_degree: float=90):
        """
        This is a function to move the manipulator to the given coordinates

        Args:
            x (float): x axis coordinate
            y (float): y axis coordinate
            z (float): z axis coordinate
            speed (int): set the speed to move from the old position to the given one
            effector_horizontal_degree (float): set the degree of effector relative to the ground, range from 0 to 180 degrees
        """

        speed = self.speed if speed == 700 and self.speed != 700 else speed
        effector_horizontal_degree = self.effector_horizontal_degree if effector_horizontal_degree == 90 and self.effector_horizontal_degree != 90 else effector_horizontal_degree

        # Calculate degrees for servo motors
        base_degree, shoulder_degree, elbow_degree, effector_degree = self.inverse_kinematics(x, y, z, effector_horizontal_degree)

        # Convert degree to microseconds
        base_angle = int(((base_degree - self.min_degree) / (self.max_degree - self.min_degree)) * (self.max_ms - self.min_ms) + self.min_ms)
        shoulder_angle = int(((shoulder_degree - self.min_degree) / (self.max_degree - self.min_degree)) * (self.max_ms - self.min_ms) + self.min_ms)
        elbow_angle = int(((elbow_degree - self.min_degree) / (self.max_degree - self.min_degree)) * (self.max_ms - self.min_ms) + self.min_ms)
        effector_angle = int(((270 - effector_degree - self.max_degree) / (self.min_degree - self.max_degree)) * (self.max_ms - self.min_ms) + self.min_ms)

        self.ser.write(f'#{self.base_pin} P{base_angle} S{speed} \
                    #{self.shoulder_pin} P{shoulder_angle} S{speed} \
                    #{self.elbow_pin} P{elbow_angle} S{speed} \
                    #{self.effector_pin} P{effector_angle} S{speed}\r'.encode())

    def move_fromto(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float, step: float=0.1, speed: int=700, effector_horizontal_degree: float=90):
        """
        This is a function to move the manipulator from the first to the second coordinates.

        Args:
            x1 (float): x1 axis coordinate
            y1 (float): y1 axis coordinate
            z1 (float): z1 axis coordinate
            x2 (float): x2 axis coordinate
            y2 (float): y2 axis coordinate
            z2 (float): z2 axis coordinate
            step (float): Steps for from the first coordinates to the second
            speed (int): set the speed to move from the old position to the given one
            effector_static_degree (float): set the degree of effector relative to the ground, range from 0 to 180 degrees
        """

        step = self.step if step == 0.1 and self.step != 0.1 else step
        speed = self.speed if speed == 700 and self.speed != 700 else speed
        effector_horizontal_degree = self.effector_horizontal_degree if effector_horizontal_degree == 90 and self.effector_horizontal_degree != 90 else effector_horizontal_degree

        x_len = abs(x1 - x2)
        y_len = abs(y1 - y2)
        z_len = abs(z1 - z2)

        max_len = max(x_len, y_len, z_len)

        x_step = x_len / max_len * step if x1 < x2 else -x_len / max_len * step
        y_step = y_len / max_len * step if y1 < y2 else -y_len / max_len * step
        z_step = z_len / max_len * step if z1 < z2 else -z_len / max_len * step

        for _ in range(int(max_len / step) + 1):
            self.move(x1, y1, z1, speed=speed, effector_horizontal_degree=effector_horizontal_degree)

            x1 += x_step
            y1 += y_step
            z1 += z_step

    def gripper(self, state: bool=False, gripper_degree: float=180, max_strength: int=20):
        """
        This is a function to move the gripper in degrees from 0-180 or take an object.

        Args:
            state (bool): Change the state to True if you only want to give a degree to the gripper
            gripper_degree (float): Is the degree for the maximum open when gripping or the angle of the gripper
            max_strength (int): This is a number from 0-255 to close the gripper, when the force reaches this number, the gripper has grabbed the object
        """

        # Maximum strength of the captured object
        max_strength = self.max_strength if max_strength == 20 and self.max_strength != 20 else max_strength  

        # Converting degree to ms
        gripper_angle = int(((gripper_degree - self.min_degree) / (self.max_degree - self.min_degree)) * (self.gripper_max_ms - self.gripper_min_ms) + self.gripper_min_ms)

        # Give gripper starting angle
        self.ser.write(f'#{self.gripper_pin} P{gripper_angle}\r'.encode())

        sleep(0.5)

        while True:
            # Reading from force sensor a value
            self.ser.write(f'V{self.force_sensor_pin}\r'.encode())
            force_sensor = int.from_bytes(self.ser.readline(), byteorder='big')

            # Condition to close gripper (grab an object)
            if not state and force_sensor < max_strength and gripper_angle > self.gripper_min_ms:
                gripper_angle -= 15
                self.ser.write(f'#{self.gripper_pin} P{gripper_angle}\r'.encode())
            else:
                break
