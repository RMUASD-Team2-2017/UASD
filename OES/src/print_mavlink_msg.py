from pymavlink import mavutil

interface = mavutil.mavlink_connection("/dev/serial0", baud=57600, autoreconnect=True)

while True:
    m = interface.recv_msg()
    if m:
        print m
        if (m.get_type() == 'HEARTBEAT'):
            pass
            print 'HEARTBEAT'
