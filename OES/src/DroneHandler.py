import threading
import time
import logging
from Queue import Queue
from dronekit import connect, VehicleMode, Command, LocationGlobal
from pymavlink import mavutil

forever = 60*60*24*365*100

logger = logging.getLogger(__name__)

#class StoppableThread(threading.Thread):
#    def __init__(self):
#        threading.Thread.__init__(self)
#        self.stop_event = threading.Event()
#
#    def stop(self):
#        if self.isAlive() == True:
#            # set event to signal thread to terminate
#            self.stop_event.set()
#            # block calling thread until thread really has terminated
#            self.join()


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
        def mode_listener(seself, attr_name, value):
            with self.class_access_hack.lock:
                self.class_access_hack.mode = value
                logger.debug('Mode %s',value)

    def get_mode(self):
        with self.lock:
            return self.mode

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


if __name__ == "__main__":
    main()
