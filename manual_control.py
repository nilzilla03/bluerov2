"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')
dt = 0.1
# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].

'''for i in range(0,int(t/dt)):
    print("forward")
    master.mav.manual_control_send(
        master.target_system,
        x=1500,
        y=0,
        z=500,
        r=0,buttons = 0)
    time.sleep(dt)

for i in range(0,int(t/dt)):
    print("backward")
    master.mav.manual_control_send(
        master.target_system,
        x=-1500,
        y=0,
        z=1000,
        r=0,buttons = 0)
    time.sleep(dt)'''

'''for i in range(0,int(t/dt)):
    print("backward")
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        z=-500,
        r=0,buttons = 0)
    time.sleep(dt)'''

t=4
for i in range(0,int(t/dt)):
    print("forward")
    master.mav.manual_control_send(
        master.target_system,
        x=1500,
        y=0,
        z=500,
        r=0,buttons = 0)
    time.sleep(dt)
t = 1
for i in range(0,int(t/dt)):
    print("backward")
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        z=500,
        r=1000,buttons = 0)
    time.sleep(dt)
t = 2
for i in range(0,int(t/dt)):
    print("backward")
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        z=-1000,
        r=1000,buttons = 0)
    time.sleep(dt)
'''t = 4
for i in range(0,int(t/dt)):
    print("forward")
    master.mav.manual_control_send(
        master.target_system,
        x=1500,
        y=0,
        z=-1000,
        r=0,buttons = 0)
    time.sleep(dt)'''
t = 1
for i in range(0,int(t/dt)):
    print("backward")
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        z=-1000,
        r=1000,buttons = 0)
    time.sleep(dt)

t=4
for i in range(0,int(t/dt)):
    print("forward")
    master.mav.manual_control_send(
        master.target_system,
        x=1500,
        y=0,
        z=-1000,
        r=0,buttons = 0)
    time.sleep(dt)
t = 1
for i in range(0,int(t/dt)):
    print("backward")
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        z=-1000,
        r=1000,buttons = 0)
    time.sleep(dt)
t = 4
for i in range(0,int(t/dt)):
    print("forward")
    master.mav.manual_control_send(
        master.target_system,
        x=1500,
        y=0,
        z=1000,
        r=0,buttons = 0)
    time.sleep(dt)


print("done")


# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
buttons = 1 + 1 << 3 + 1 << 7
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    500, # 500 means neutral throttle
    0,
    buttons)