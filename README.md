# Robotic Arm Control with Python

## Project overview

This project was created for learning and practical testing of a robotic arm manipulator. The main idea is to show how Python code, servo motors, inverse kinematics, serial communication, and the SSC 32U board can work together in one system.

The arm can move to a selected position using x, y, and z coordinates. The program calculates the needed servo angles and sends movement commands to the SSC 32U board. The gripper can also open, close, and hold an object.

## Hardware used

- SSC 32U servo controller board
- Servo motors for the arm joints
- Servo motor for the gripper
- Force sensor
- 3D printed arm parts
- Power supply for the servo motors
- Computer with Python installed

## Python libraries used

`math`, `serial`, `time`  
Only these libraries are used so the code stays simple and easy to understand.

## Servo connection

- Base servo is connected to channel `0`
- Shoulder servo is connected to channel `1`
- Elbow servo is connected to channel `16`
- End effector servo is connected to channel `17`
- Gripper servo is connected to channel `18`
- Force sensor is connected to analog input `A`

## Main features

- Move the robotic arm to a selected coordinate
- Calculate joint angles using inverse kinematics
- Convert angles to servo pulse values
- Send commands to the SSC 32U board
- Move the arm from one position to another position
- Open and close the gripper
- Use force sensor data while grabbing an object

## How it works

The user gives a target position for the robotic arm. The program uses math calculations to find the needed angles for the arm joints. After that, the angles are converted into servo pulse values. These values are sent through the serial port to the SSC 32U board. The board then moves the servo motors.

The gripper can be controlled separately. It can close step by step and use the force sensor value to stop when the object is held.

## Basic usage

First connect the SSC 32U board to the computer.

Then connect the servo motors to the correct channels.

After that, open the Python file and set the correct serial port.

Example for Windows

```python
from RoboticArm import RAConfig

arm = RAConfig("COM3")
arm.move(15, 10, 20)
arm.gripper()
```

Example for macOS

```python
from RoboticArm import RAConfig

arm = RAConfig("/dev/tty.usbserial")
arm.move(15, 10, 20)
arm.gripper()
```

## Movement example

```python
arm.move(x=20, y=5, z=15)
```

This command moves the arm to the selected coordinate.

## Move from one point to another

```python
arm.move_fromto(x1=10, y1=5, z1=15, x2=20, y2=10, z2=18)
```

This command moves the arm from the first coordinate to the second coordinate.

## Gripper example

```python
arm.gripper()
```

This command closes the gripper while checking the force sensor value.

## Manual gripper angle

```python
arm.gripper(state=True, gripper_degree=90)
```

This command moves the gripper to a selected angle.

## Notes

- Check the power supply before testing the arm.
- Check the serial port name before running the program.
- Check all servo channel connections.
- Start testing with small movements.
- Do not force the arm outside its real mechanical limits.
- If the arm moves in the wrong direction, check the servo position and channel connection.

## Result

This project shows a simple way to control a robotic arm with Python and the SSC 32U board. It is useful for learning robotic arm movement, servo control, serial communication, and basic inverse kinematics.
