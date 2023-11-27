#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function 
import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1 
#from pymavlink.dialects.v20 import ardupilotmega as mavlink2 

# DO NOT USE THIS CODE DIRECTLY ON A REAL ROBOT WITHOUT CHECKING WHAT IT MIGHT DO!

# See https://www.ardusub.com/developers/pymavlink.html and https://mavlink.io/en/messages/common.html

#autopilot = mavutil.mavlink_connection("COM12", 115200)
#autopilot = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
autopilot = mavutil.mavlink_connection('udp:0.0.0.0:14550')
autopilot.wait_heartbeat()
n = 4; dt = 0.1 # To send repeatedly the messages in case of unreliable network
# Neutral joystick...
for i in range(0,n):
    autopilot.mav.manual_control_send(autopilot.target_system, 0, 0, 500, 0, 0)
    time.sleep(dt)
# Arm
print("Arming")
for i in range(0,n):
    autopilot.mav.command_long_send(autopilot.target_system, autopilot.target_component, mavlink1.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(dt)
time.sleep(3)
# Stabilize (0) mode...
# See enum control_mode_t in https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/defines.h
'''for i in range(0,n):
    autopilot.mav.set_mode_send(autopilot.target_system, mavlink1.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    time.sleep(dt)
time.sleep(1)'''
# Move
print("Forward")
t = 4
for i in range(0,int(t/dt)):
    autopilot.mav.manual_control_send(autopilot.target_system, x=1000, y=0, z=500, r=0, buttons=0)
    #ime.sleep(dt)
print("Turn")
t = 0.5
for i in range(0,int(t/dt)):
    autopilot.mav.manual_control_send(autopilot.target_system, 0, 0, 500, -1000, 0, 0)
    #time.sleep(dt)
print("Submerge")
t = 4
for i in range(0,int(t/dt)):
    autopilot.mav.manual_control_send(autopilot.target_system, 0, 0, 200, 0, 0, 0) # Does not work with SITL...?
    #time.sleep(dt)
print("Stop")
for i in range(0,n):
    autopilot.mav.manual_control_send(autopilot.target_system, 0, 0, 500, 0, 0, 0)
    #time.sleep(dt)
#time.sleep(2)
autopilot.close()