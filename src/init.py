#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *
import PyDynamixel_v2 as pd
import cv2
from time import sleep

# Declare the DxlComm variable
port = "/dev/ttyUSB0"
baudrate = 57600
serial = pd.DxlComm(port=port, baudrate=baudrate)

# Declare a dynamixel joint
dyn_id1 = 1
dyn_id2 = 2
dyn_id3 = 3
dyn_id4 = 4
dyn_id5 = 5
dyn_id6 = 6
dyn_id7 = 19
dyn_id8 = 20

dyn1 = pd.Joint(dyn_id1)
dyn2 = pd.Joint(dyn_id2)
dyn3 = pd.Joint(dyn_id3)
dyn4 = pd.Joint(dyn_id4)
dyn5 = pd.Joint(dyn_id5)
dyn6 = pd.Joint(dyn_id6)
dyn7 = pd.Joint(dyn_id7)
dyn8 = pd.Joint(dyn_id8)

# Attach this joint to DxlComm to enable serial communication
print("Attach joint")
serial.attach_joints([dyn1, dyn2, dyn3, dyn4, dyn5, dyn6, dyn7, dyn8])
# You could also send all joints as a list to DxlComm varible
# serial.attach_joints([dyn, dyn2, dyn3])


# Enable single joint torque or all joints torques
print("Enable torque")
serial.enable_torques()

sleep(2)

def zero():
    print('Send all motors to zero position')
    dyn1.send_angle(190)
    dyn2.send_angle(190)
    dyn3.send_angle(210)
    dyn4.send_angle(210)
    dyn5.send_angle(80)
    dyn6.send_angle(80)
    dyn7.send_angle(180)
    dyn8.send_angle(180)

    print("\n----------------------------------------------------" +
        "\nID 1 - Current angle: {}\n".format(dyn1.get_angle()) +
        "ID 2 - Current angle: {}\n".format(dyn2.get_angle()) +
        "ID 3 - Current angle: {}\n".format(dyn3.get_angle()) +
        "ID 4 - Current angle: {}\n".format(dyn4.get_angle()) +
        "ID 5 - Current angle: {}\n".format(dyn5.get_angle()) +
        "ID 6 - Current angle: {}\n".format(dyn6.get_angle()) +
        "ID 19 - Current angle: {}\n".format(dyn7.get_angle()) +
        "ID 20 - Current angle: {}\n".format(dyn8.get_angle()) +
        "----------------------------------------------------\n" 
        )
    


def sync_read():
    # Sync Read of all attached joints. Only in Protocol 2.0
    print("Sync read test")
    angles = serial.get_angles()
    print(angles)

def all_pings():
    # Broadcast ping only available for Protocol 2.0
    serial.broadcast_ping()

sync_read()
all_pings()
sleep(5)
zero()
print("Chamou")


key = cv2.waitKey(1) & 0xFF

# if the 'q' key is pressed, stop the loop
if key == ord("d"):
    # Disable all or single joints torques
    print("Disable all top torques")
    serial.disable_torques() # all joints

if key == ord("e"):
    # Disable all or single joints torques
    print("Enable all top torques")
    serial.enable_torques() # all joints

# Close serial port
serial.release()
