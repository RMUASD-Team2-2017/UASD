import threading
import time
import logging
from Queue import Queue
from dronekit import connect, VehicleMode, Command, LocationGlobal
from pymavlink import mavutil
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

    def __init__(self,port, signal_queue, baud=None):
        StoppableThread.__init__(self)
        self.signal = signal_queue
        self.heartbeat = None
        self.heartbeat_lock = threading.Lock()
        self.position_lock = threading.Lock()
        self.mode_lock = threading.Lock()
        self.state_lock = threading.Lock()

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
        self.hi_control = io()

        if baud is None:
            self.mav_interface = mavutil.mavlink_connection(port, autoreconnect=True)
        else:
            self.mav_interface = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)

    def run(self):
        while self.stop_event.is_set() is False:

            # Receive messages
            m = self.mav_interface.recv_msg()
            if m:
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

                # Update completion state - only relevant when we have got new information
                self.completion_statemachine()
            # Avoid busy loop
            time.sleep(0.001)

    def completion_statemachine(self):
        with self.state_lock:
            if self.state is DroneHandler_pymavlink.STATE_IDLE:
                self.hi_control.hi_safe()
                if self.armed is True:
                    self.state = DroneHandler_pymavlink.STATE_TAKEOFF
                    self.hi_control.hi_landing_siren()
                    logger.info('State: Takeoff')
                    self.request_mission_list() # Request number of waypoints. Gives a timeout because we never request the waypoints
            elif self.state is DroneHandler_pymavlink.STATE_TAKEOFF:
                if self.current_waypoint > 0:
                    self.state = DroneHandler_pymavlink.STATE_ENROUTE
                    self.hi_control.hi_flash_rotation_indicators_siren()
                    logger.info('State: Enroute')
            elif self.state is DroneHandler_pymavlink.STATE_ENROUTE:
                if self.current_waypoint == self.number_of_waypoints-1:
                    self.state = DroneHandler_pymavlink.STATE_LANDING
                    self.hi_control.hi_landing_siren()
                    logger.info('State: Landing')
            elif self.state is DroneHandler_pymavlink.STATE_LANDING:
                if self.armed is False:
                    self.state = DroneHandler_pymavlink.STATE_LANDED
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
    drone_handler = DroneHandler_pymavlink('127.0.0.1:14540',drone_handler_signal_queue)
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

if __name__ == "__main__":
    main_pymavlink()
