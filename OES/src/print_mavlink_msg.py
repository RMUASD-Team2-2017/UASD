from pymavlink import mavutil

interface = mavutil.mavlink_connection("/dev/ttyUSB0", baud=921600, autoreconnect=True)

while True:
    m = interface.recv_msg()
    if m:
        print m
        if (m.get_type() == 'HEARTBEAT'):
            print 'HEARTBEAT'
            interface.mav.set_position_target_local_ned_send(
            0,       # time_boot_ms (not used)
            1, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


#            print 'Mode:', mavutil.interpret_px4_mode(m.base_mode, m.custom_mode), 
            # print m

