import threading
import time
import logging
from GpsMonitor import GpsMonitor

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


class OnboardControl(StoppableThread):
    HEARTBEAT_LOST_TIME = 3
    CONNECTION_LOST = 'CONNECTION_LOST'
    CONNECTION_OK = 'CONNECTION_OK'

    def __init__(self,drone_handler, drone_handler_signal_queue,rate = 1):
        StoppableThread.__init__(self)
        self.name = 'OnboardControl'
        self.rate = float(rate)
        self.drone_handler = drone_handler
        self.signal_gps_lock = threading.Lock()
        self.signal_drone_heartbeat_lock = threading.Lock()
        self.signal_gsm_lock = threading.Lock()
        self.gps_state = None
        self.gps_source = None
        self.drone_heartbeat = time.time()
        self.drone_connection_state = None
        self.gsm_connection_state = None

        self.signal_queue = drone_handler_signal_queue

        self.action_list_terminate = []
        self.action_list_land_here = [GpsMonitor.STATE_LOST]
        self.action_list_return_to_launch = [OnboardControl.CONNECTION_LOST]
        self.action_list_wait_here = [GpsMonitor.STATE_TIMEOUT,
                                      GpsMonitor.STATE_MISMATCH]
        self.action_list_no_action = [GpsMonitor.STATE_OK,
                                      GpsMonitor.STATE_NO_FIX_YET,
                                      OnboardControl.CONNECTION_OK]

        self.terminate_activated = False
        self.land_here_activated = False
        self.return_to_launch_activated = False
        self.wait_here_activated = False

        self.timer_wait_here = None

    def signal_gps_state(self, state, source = None):
        with self.signal_gps_lock:
            self.gps_state = state
            self.gps_source = source

    def signal_drone_state(self,state):
        with self.signal_drone_heartbeat_lock:
            self.drone_heartbeat = state

    def signal_gsm_state(self,state):
        with self.signal_gsm_lock:
            self.gsm_connection_state = state

    def run(self):
        logger.info('OnboardControl started')
        while self.stop_event.is_set() is False:

            # Update heartbeat
            while not self.signal_queue.empty():
                self.drone_heartbeat = self.signal_queue.get()

            # Monitor heartbeat from dronekit
            # Lock is not required, just kept for remembrance if we should change something
            with self.signal_drone_heartbeat_lock:
                if time.time() - self.drone_heartbeat > OnboardControl.HEARTBEAT_LOST_TIME:
                    self.drone_connection_state = OnboardControl.CONNECTION_LOST
                else:
                    self.drone_connection_state = OnboardControl.CONNECTION_OK

            # Severity order: Terminate, land here, return to launch, wait 10 seconds
            if self.check_states(self.action_list_terminate):
                if not self.terminate_activated:
                    logger.info('Action: Terminate flight')
                    self.stop_wait_here_timer()
                    self.terminate_activated = True # We can never enter this again
                    self.drone_handler.terminate_flight()
            elif self.check_states(self.action_list_land_here):
                if not self.land_here_activated and \
                   not self.terminate_activated:
                    logger.info('Action: Land here')
                    self.stop_wait_here_timer()
                    self.land_here_activated = True # We can never enter this again
                    self.drone_handler.land_at_current_location()
            elif self.check_states(self.action_list_return_to_launch):
                if not self.return_to_launch_activated and \
                   not self.terminate_activated and \
                   not self.land_here_activated:
                    logger.info('Action: Return to launch')
                    self.stop_wait_here_timer()
                    self.return_to_launch_activated = True # We can never enter this again
                    self.drone_handler.return_to_launch()
            elif self.check_states(self.action_list_wait_here):
                if not self.wait_here_activated and \
                   not self.terminate_activated and \
                   not self.land_here_activated and \
                   not self.land_here_activated:
                    logger.info('Action: Wait here')
                    self.wait_here_activated = True
                    self.drone_handler.loiter()
                    self.timer_wait_here = threading.Timer(10.0, self.disable_wait_here)
                    self.timer_wait_here.start()

            elif self.check_states(self.action_list_no_action):
                logger.info('Action: None')
                pass  # No-op
            else:
                logger.info('Action: No states set yet')

            # Sleep to obtain desired rate
            time.sleep(1.0/self.rate)

        self.stop_wait_here_timer()
        logger.info('OnboardControl terminating')

    def check_state(self,state,action_list):
        return state in action_list

    def check_states(self,action_list):
        ret = (self.check_state(self.gps_state,action_list) or \
               self.check_state(self.drone_connection_state, action_list) or \
               self.check_state(self.gsm_connection_state, action_list))
        return ret

    def disable_wait_here(self):
        self.drone_handler.resume_mission()
        self.timer_wait_here = None
        self.wait_here_activated = False
        logger.info('wait_here state reenabled')

    def stop_wait_here_timer(self):
        if self.timer_wait_here and self.timer_wait_here.is_alive():
            self.timer_wait_here.cancel()
            logger.info('wait_here_timer cancelled')