import threading
import time
import logging
from Queue import Queue
from dronekit import connect, VehicleMode, Command, LocationGlobal
from pymavlink import mavutil, mavwp

ON_PI = True
if ON_PI:
    from RPiGpio import io

forever = 60*60*24*365*100

logger = logging.getLogger(__name__)

class StoppableThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()

    def stop(self):
        if self.isAlive() == True:
            # set event to signal thread to terminate
            self.stop_event.set()
            # block calling thread until thread really has terminated
            self.join()


class DroneHandler():
    STATE_OK = 'HEARTBEAT_OK'
    STATE_TIMEOUT = 'HEARTBEAT_TIMEOUT'
    MANUAL_MODE = 'MANUAL'
    GPS_FIX_TYPE_2D_FIX = 2
    GPS_FIX_TYPE_3D_FIX = 3
    def __init__(self,port,baud,signal_queue):
        # TODO: Add baud= for a real drone
        # Connect to the drone
        #   Try to reconnect forever (100 years)
        #   Update rate 4 Hz
        #   wait_ready=True: Block connect until all mandatory parameters are fetched
        self.vehicle = connect(port, baud=baud, rate=4, wait_ready=False, source_system=1,heartbeat_timeout=forever)
        self.position = None
        # Dronekit does not seem to be thread safe. Lock everything just in case
        #   Ref: https://groups.google.com/forum/#!msg/drones-discuss/PvgF7AiYLmI/sw4BuglTHAAJ
        self.lock = threading.Lock()
        self.vehicle.class_access_hack = self # Inspired by group 1. Hack to access class from decorator callbacks
        self.signal = signal_queue
        self.mode = None
        self.home_position = None

        logger.info('DroneHandler started')

        @self.vehicle.on_message('GPS_RAW_INT')
        def listener(self, name, message):
            with self.class_access_hack.lock:
                position = {
                    'lat': self.location.global_relative_frame.lat,
                    'lng': self.location.global_relative_frame.lon,
                    'alt': self.location.global_relative_frame.alt
                }
                # DOP values are in cm
                dop = {'HDOP': self.gps_0.eph,
                       'VDOP': self.gps_0.epv}
                self.class_access_hack.position = {'timestamp': time.time(),
                                 'position': position,
                                 'fix_type': self.gps_0.fix_type,
                                 'DOP': dop,
                                 'satellites': self.gps_0.satellites_visible
                                 }
                if self.class_access_hack.home_position is None \
                        and ( self.class_access_hack.mode == None \
                        or self.class_access_hack.mode == 'MANUAL')\
                        and self.gps_0.fix_type > DroneHandler.GPS_FIX_TYPE_2D_FIX:
                    self.class_access_hack.home_position = position

        @self.vehicle.on_message('HEARTBEAT')
        def listener(self, name, message):
            with self.class_access_hack.lock:
                self.class_access_hack.signal.put(time.time())
                self.class_access_hack.last_heartbeat = time.time()

        @self.vehicle.on_attribute('mode')
        def mode_listener(self, attr_name, value):
            with self.class_access_hack.lock:
                self.class_access_hack.mode = value
                logger.debug('Mode %s',value)

    def get_mode(self):
        with self.lock:
            #return self.mode
            return self.vehicle.mode

    def get_position(self):
        with self.lock:
            return self.position

    def get_home_position(self):
        with self.lock:
            return self.home_position

    def get_heartbeat(self):
        with self.lock:
            return self.last_heartbeat

    def terminate_flight(self):
        with self.lock:
            # This should probably be improved by going to a low altitude first
            self.vehicle.armed = False

    def return_to_launch(self):
        with self.lock:
            self.vehicle.mode = VehicleMode("RTL")

    def land_at_current_location(self):
        with self.lock:
            self.vehicle.mode = VehicleMode("LAND")

    def loiter(self):
        with self.lock:
            self.vehicle.mode = VehicleMode("LOITER")

    def resume_mission(self):
        with self.lock:
            self.vehicle.mode = VehicleMode("MISSION")

    def close(self):
        self.vehicle.close()
        logger.info('DroneHandler terminating')


def convert_from_1e7(number):
    return convert_from_1ex(number, 7)


def convert_from_1ex(number,exponent):
    return float(number)/float(10**exponent)


class DroneHandler_pymavlink(StoppableThread):
    STATE_OK = 'HEARTBEAT_OK'
    STATE_TIMEOUT = 'HEARTBEAT_TIMEOUT'
    MANUAL_MODE = 'MANUAL'
    GPS_FIX_TYPE_2D_FIX = 2
    GPS_FIX_TYPE_3D_FIX = 3
    ARMED_FLAG = 0b10000000
    STATE_IDLE = 0
    STATE_TAKEOFF = 1
    STATE_ENROUTE = 2
    STATE_LANDING = 3
    STATE_LANDED = 4

    def __init__(self,port, signal_queue, gsm_transmit_queue, baud=None):
        StoppableThread.__init__(self)
        self.signal = signal_queue
        self.gsm_transmit_queue = gsm_transmit_queue
        self.heartbeat = None
        self.heartbeat_lock = threading.Lock()
        self.position_lock = threading.Lock()
        self.mode_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.waypoints_lock = threading.Lock()

        self.mav_interface = None
        self.mode = None
        self.position = None
        self.home_position = None
        self.current_waypoint = None
        self.armed = None
        self.current_waypoint = None
        self.number_of_waypoints = None
        self.request_mission_list_sent = False
        self.state = DroneHandler_pymavlink.STATE_IDLE
        if ON_PI:
            self.hi_control = io()
        self.idle_output_counter = 0
        self.waypoints = mavwp.MAVWPLoader()
        self.clearing_waypoints = threading.Event()


        if baud is None:
            self.mav_interface = mavutil.mavlink_connection(port, autoreconnect=True)
        else:
            self.mav_interface = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)

    def run(self):
        logger.info(' Started')

        while self.stop_event.is_set() is False:

            # Receive messages
            m = self.mav_interface.recv_msg()
            if m:
                #print m
                if m.get_type() == 'HEARTBEAT':
                    with self.heartbeat_lock:
                        self.heartbeat = time.time()
                    self.signal.put(time.time())
                    with self.mode_lock:
                        self.mode = mavutil.interpret_px4_mode(m.base_mode, m.custom_mode)
                    if m.base_mode & DroneHandler_pymavlink.ARMED_FLAG > 0: # Check if the drone is armed
                        self.armed = True
                    else:
                        self.armed = False
                if m.get_type() == 'GPS_RAW_INT':
                    msg = m.to_dict()
                    position = {
                        'lat': convert_from_1e7(msg['lat']),
                        'lng': convert_from_1e7(msg['lon']),
                        'alt': convert_from_1ex(msg['alt'],3)
                    }
                    dop = {'HDOP': msg['eph'],
                           'VDOP': msg['epv']}
                    with self.position_lock:
                        self.position = {'timestamp': time.time(),
                                         'position': position,
                                         'fix_type': msg['fix_type'],
                                         'DOP': dop,
                                         'satellites': msg['satellites_visible']
                                         }
                        if self.home_position is None \
                                and msg['fix_type'] > DroneHandler.GPS_FIX_TYPE_2D_FIX:
                            logger.debug(' Home position set')
                            self.home_position = position
                if m.get_type() == 'MISSION_CURRENT':
                    self.current_waypoint = m.seq

                if m.get_type() == 'MISSION_COUNT':
                    self.number_of_waypoints = m.count

                if m.get_type() == 'MISSION_REQUEST':
                    #Transmit requested waypoint
                    print m
                    with self.waypoints_lock:
                        print self.waypoints.wp(m.seq)
                        self.mav_interface.mav.send(self.waypoints.wp(m.seq))
                if m.get_type() == 'MISSION_ACK':
                    # We get this after complete mission waypoint upload and clearing
                    print m

                    # Check if it was because of waypoint clearing or upload
                    if self.clearing_waypoints.is_set():
                        self.clearing_waypoints.clear()
                    else:
                        msg = {'type': 'MISSION_RESULT', 'value': False}
                        if m.type == 0:
                            msg['value'] = True
                        print msg
                        self.gsm_transmit_queue.put(msg)

                # Update completion state - only relevant when we have got new information
                self.completion_statemachine()
            # Avoid busy loop
            time.sleep(0.001)

    def completion_statemachine(self):
        with self.state_lock:
            if self.state is DroneHandler_pymavlink.STATE_IDLE:
                if ON_PI:
                    self.hi_control.hi_safe()
                self.idle_output_counter += 1
                if  self.idle_output_counter == 100:
                    logger.info('State: Idle')
                    self.idle_output_counter = 0
                if self.armed is True:
                    self.state = DroneHandler_pymavlink.STATE_TAKEOFF
                    if ON_PI:
                        self.hi_control.hi_landing_siren()
                    logger.info('State: Takeoff')
                    self.request_mission_list() # Request number of waypoints. Gives a timeout because we never request the waypoints
            elif self.state is DroneHandler_pymavlink.STATE_TAKEOFF:
                if self.current_waypoint > 0:
                    self.state = DroneHandler_pymavlink.STATE_ENROUTE
                    if ON_PI:
                        self.hi_control.hi_flash_rotation_indicators_siren()
                    logger.info('State: Enroute')
            elif self.state is DroneHandler_pymavlink.STATE_ENROUTE:
                if self.current_waypoint == self.number_of_waypoints-1:
                    self.state = DroneHandler_pymavlink.STATE_LANDING
                    if ON_PI:
                        self.hi_control.hi_landing_siren()
                    logger.info('State: Landing')
            elif self.state is DroneHandler_pymavlink.STATE_LANDING:
                if self.armed is False:
                    self.state = DroneHandler_pymavlink.STATE_LANDED
                    if ON_PI:
                        self.hi_control.hi_safe()
                    logger.info('State: Landed')
            elif self.state is DroneHandler_pymavlink.STATE_LANDED:
                pass

    def request_mission_list(self):
        self.mav_interface.mav.mission_request_list_send(
            1,  # target system
            0,  # target component
            0)  # Mission type (0: Mission)

    # Api functions #
    def get_heartbeat(self):
        with self.heartbeat_lock:
            return self.heartbeat

    def get_mode(self):
        with self.mode_lock:
            return self.mode

    def get_position(self):
        with self.position_lock:
            return self.position

    def get_home_position(self):
        with self.position_lock:
            return self.home_position

    def terminate_flight(self):
        logger.info('Terminate flight')
        self.mav_interface.arducopter_disarm()

    def return_to_launch(self):
        logger.info('RTL')
        self.mav_interface.set_mode('RTL')

    def land_at_current_location(self):
        logger.info('Land')
        self.mav_interface.set_mode('LAND')

    def loiter(self):
        logger.info('Loiter')
        self.mav_interface.set_mode('LOITER')

    def resume_mission(self):
        logger.info('Resume mission')
        self.mav_interface.set_mode('MISSION')

    def get_state(self):
        with self.state_lock:
            return self.state

    def upload_mission(self,mission):
        # ROS constants
        SYS_ID = 1
        COMP_ID = 1
        ROS_TAKEOFF = 0
        ROS_WAYPOINT = 1
        ROS_LAND = 2

        # Mission constants
        HOLD_TIME = 0.0
        ACCEPTANCE_RADIUS = 5.0


        with self.waypoints_lock:
            # Based on https://gist.github.com/donghee/8d8377ba51aa11721dcaa7c811644169
            logger.info('Upload mission')
            path = mission['path']
            print mission
            print path

            # Clear waypoints in uploader
            self.waypoints.clear()

            # Add waypoints to the uploader
            for waypoint in enumerate(path):
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                seq = waypoint[0]
                lat = waypoint[1]['lat']
                lon = waypoint[1]['lon']
                altitude = waypoint[1]['alt']
                autocontinue = 1
                current = 0
                param1 = 0
                param2 = 0
                param3 = 0
                param4 = 0
                if waypoint[1]['type'] == ROS_TAKEOFF:
                    command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

                elif waypoint[1]['type'] == ROS_WAYPOINT:
                    command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                    param1 = HOLD_TIME
                    param2 = ACCEPTANCE_RADIUS
                    param3 = 0
                    param4 = 0
                elif waypoint[1]['type'] == ROS_LAND:
                    command = mavutil.mavlink.MAV_CMD_NAV_LAND
                p = mavutil.mavlink.MAVLink_mission_item_message(SYS_ID, COMP_ID, seq, frame, command, current, autocontinue,
                                                                 param1, param2, param3, param4, lat, lon, altitude)
                # Add the point
                self.waypoints.add(p)

            # Clear current waypoints
            self.clearing_waypoints.set()
            self.mav_interface.waypoint_clear_all_send()

            # Allow the flight controller to receive it
            time.sleep(1)

            #Publish the number of waypoints to start upload
            self.mav_interface.waypoint_count_send(self.waypoints.count())


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    logger.info('Started')
    drone_handler_signal_queue = Queue()
    drone_handler = DroneHandler('127.0.0.1:14540',115200,drone_handler_signal_queue)
    while True:
        time.sleep(1)
        print 'Mode', drone_handler.get_mode()
        print 'Position', drone_handler.get_position()
        print 'Home position', drone_handler.get_home_position()


def main_pymavlink():
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Started')
    drone_handler_signal_queue = Queue()
    drone_handler = DroneHandler_pymavlink('/dev/serial0',drone_handler_signal_queue, baud=57600)
    drone_handler.start()

    do_exit = False
    while do_exit == False:
        try:
            time.sleep(1)
            #print 'Mode', drone_handler.get_mode()
            #print 'Position', drone_handler.get_position()
            #print 'Home position', drone_handler.get_home_position()
            #drone_handler.resume_mission()
            #print drone_handler.get_state()
        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True
    drone_handler.stop()


def test_mission_upload():
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Started')
    drone_handler_signal_queue = Queue()
    gsm_transmit_queue = Queue()
    drone_handler = DroneHandler_pymavlink('127.0.0.1:14540', drone_handler_signal_queue, gsm_transmit_queue, baud=57600)
    drone_handler.start()

    mission = {'path': [{'lat': 0.0, 'lon': 0.0, 'alt': 15.0, 'type': 0}, {'lat': 55.395301818847656, 'lon': 10.37147045135498, 'alt': 15.0, 'type': 1}, {'lat': 55.395301818847656, 'lon': 10.37147045135498, 'alt': 0.0, 'type': 2}], 'type': u'MISSION'}
    print 'Upload succes:', drone_handler.upload_mission(mission)

    do_exit = False
    while do_exit == False:
        try:
            if not gsm_transmit_queue.empty():
                print gsm_transmit_queue.get()
            time.sleep(1)

        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True
    drone_handler.stop()

if __name__ == "__main__":
    main_pymavlink()
