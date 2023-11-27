from pymavlink import mavutil
import time, threading

hbRate = mavutil.periodic_event(1) # 1Hz 
cmdRate = mavutil.periodic_event(1)

# Sets the target depth while in depth-hold mode
# Target must be negative
def setTargetDepth(target):
    mavConnection.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - bootTime)), # ms since boot
        mavConnection.target_system, mavConnection.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=( # ignore everything except z position
            #mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            #mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=target, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

# Set the vehicle attitude
# Takes values in the range [-100, 100]    
# MAVLink message : https://mavlink.io/en/messages/common.html#MANUAL_CONTROL 
def setAttitude(roll, pitch, yaw, thrust):
    if(roll in range(-100,100)) and (pitch in range(-100,100)) and (yaw in range(-100,100)) and (thrust in range(-100,100)):
        # Normalize [-100, 100] in the range [-1000, 1000]
        x_norm = pitch * 10
        y_norm = roll * 10
        r_norm = yaw * 10
        # Normalize [-100, 100] in the range [0, 1000]
        z_norm = int(((thrust  + 100) / 200) * 1000)
        mavConnection.mav.manual_control_send(
            mavConnection.target_system,
            x_norm,
            y_norm,
            z_norm,
            r_norm,
            0
        )

def heartbeatLoop():
    while True:
        if hbRate.trigger():
            mavConnection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0
            )

def cmdLoop():
    while True:
        if cmdRate.trigger():
            setTargetDepth(-1)
            setAttitude(0, 20, 0, 0)

            
# Connect to the vehicle
#mavConnection = mavutil.mavlink_connection('udpin:localhost:14552') # SITL
mavConnection = mavutil.mavlink_connection('udpout:0.0.0.0:14550') # BlueRov
msg = None
elapsed = 0
timeout = 10
while not msg:
    mavConnection.mav.ping_send(
        int(time.time() * 1e6),
        0,
        0,
        0
    )
    msg = mavConnection.recv_match()
    time.sleep(0.5)
    elapsed += 0.5
    if elapsed >= timeout:
        print("Failed to connect")
        exit(1)

bootTime = time.time()

mavConnection.arducopter_disarm()
mavConnection.motors_disarmed_wait()

# Start sending heartbeat
hbThread = threading.Thread(target= heartbeatLoop, daemon= True)
hbThread.start()

# Set depth hold mode
map = None
while map==None:
    map = mavConnection.mode_mapping()

DEPTH_HOLD_MODE = map['ALT_HOLD']
while not mavConnection.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    mavConnection.set_mode('ALT_HOLD')

# Send depth and position commands
mavConnection.arducopter_arm()
mavConnection.motors_armed_wait()

cmdLoop()