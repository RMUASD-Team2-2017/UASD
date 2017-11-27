import threading
import time
import logging
from DroneHandler import DroneHandler, DroneHandler_pymavlink
from GpsMonitor import GpsMonitor
from ConnectionMonitor import ConnectionMonitor

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

    def __init__(self,drone_handler, drone_handler_signal_queue, gsm_transmit_queue, gsm_command_queue, rate = 1):
        StoppableThread.__init__(self)
        self.name = 'OnboardControl'
        self.rate = float(rate)
        self.drone_handler = drone_handler
        self.gsm_transmit_queue = gsm_transmit_queue
        self.gsm_command_queue = gsm_command_queue
        self.signal_gps_lock = threading.Lock()
        self.signal_drone_heartbeat_lock = threading.Lock()
        self.signal_gsm_lock = threading.Lock()
        self.signal_connection_state_lock = threading.Lock()
        self.ready_lock = threading.Lock()
        self.gps_state = None
        self.gps_source = None
        self.drone_heartbeat = time.time()
        self.drone_connection_state = None
        self.gsm_connection_state = None
        self.connection_state = None
        self.command_terminate = False
        self.command_land_here = False
        self.command_return_to_launch = False
        self.ready # Ready state to transmit over gsm

        self.signal_queue = drone_handler_signal_queue

        self.action_list_terminate = [] # ConnectionMonitor.STATE_CON_SERIAL2_LOST should be here... I don't dare it yet
        self.action_list_land_here = [GpsMonitor.STATE_LOST, GpsMonitor.STATE_GEOFENCE_BREACH]
        self.action_list_return_to_launch = [OnboardControl.CONNECTION_LOST,
                                             ConnectionMonitor.STATE_CON_TELEMETRY_OK_GSM_LOST,
                                             ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_LOST,
                                             ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_OK,
                                             ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_TIMEOUT,
                                             ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_LOST]
        self.action_list_wait_here = [GpsMonitor.STATE_TIMEOUT,
                                      GpsMonitor.STATE_MISMATCH,
                                      ConnectionMonitor.STATE_CON_TELEMETRY_OK_GSM_TIMEOUT,
                                      ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_OK,
                                      ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_TIMEOUT]
        self.action_list_no_action = [GpsMonitor.STATE_OK,
                                      GpsMonitor.STATE_NO_FIX_YET,
                                      OnboardControl.CONNECTION_OK,
                                      ConnectionMonitor.STATE_CON_OK]

        self.terminate_activated = False
        self.land_here_activated = False
        self.return_to_launch_activated = False
        self.wait_here_activated = False

        self.timer_wait_here = None

    def signal_gps_state(self, state, source = None):
        with self.signal_gps_lock:
            self.gps_state = state
            self.gps_source = source

    def signal_drone_state(self, state):
        with self.signal_drone_heartbeat_lock:
            self.drone_heartbeat = state

    def signal_gsm_state(self, state):
        with self.signal_gsm_lock:
            self.gsm_connection_state = state

    def signal_connection_state(self, state):
        with self.signal_connection_state_lock:
            self.connection_state = state

    def get_readiness(self):
        with self.ready_lock:
            return self.ready

    def run(self):
        logger.info('OnboardControl started')
        while self.stop_event.is_set() is False:

            # Update heartbeat
            while not self.signal_queue.empty():
                self.drone_heartbeat = self.signal_queue.get()

            while not self.gsm_command_queue.empty():
                msg = self.gsm_command_queue.get()
                if msg['type'] == 'ACTION_TERMINATE':
                    self.command_terminate = True
                    logger.info('GSM_COMMAND: Terminate')
                elif msg['type'] == 'ACTION_LAND_HERE':
                    self.command_land_here = True
                    logger.info('GSM_COMMAND: Land here')
                elif msg['type'] == 'ACTION_RETURN_TO_LAUNCH':
                    self.command_return_to_launch = True
                    logger.info('GSM_COMMAND: Return to launch')

            # We should never override manual mode
            mode = self.drone_handler.get_mode()
            if mode == DroneHandler.MANUAL_MODE or mode == None or state == DroneHandler:
                logger.debug("NO ONBOARD CONTROL: Manual mode or no mode yet")
                # Sleep to obtain desired rate
                time.sleep(1.0 / self.rate)
                # Skip the rest of the loop and start over
                continue

            # We should not override while on ground or when we have completed the mission
            # We should also check if we are in an acceptable state to start a mission
            state = self.drone_handler.get_state()
            if state == DroneHandler_pymavlink.STATE_IDLE or state == DroneHandler_pymavlink.STATE_LANDED:

                terminate = self.check_states(self.action_list_terminate)
                land_here = self.check_states(self.action_list_land_here)
                return_to_launch = self.check_states(self.action_list_return_to_launch)
                wait_here = self.check_states(self.action_list_wait_here)
                self.ready = terminate and land_here and return_to_launch and wait_here # This should be captured and sent by the gsm thread on request

                time.sleep(1.0 / self.rate)
                continue

            # Monitor heartbeat from dronekit
            # Lock is not required, just kept for remembrance if we should change something
            # This should probably be moved to connection monitor
            #with self.signal_drone_heartbeat_lock:
            #    if time.time() - self.drone_heartbeat > OnboardControl.HEARTBEAT_LOST_TIME:
            #        self.drone_connection_state = OnboardControl.CONNECTION_LOST
            #    else:
            #        self.drone_connection_state = OnboardControl.CONNECTION_OK

            # Severity order: Terminate, land here, return to launch, wait 10 seconds
            if self.check_states(self.action_list_terminate) or self.command_terminate:
                if not self.terminate_activated:
                    logger.info('Action: Terminate flight')
                    self.stop_wait_here_timer()
                    self.terminate_activated = True # We can never enter this again
                    self.drone_handler.terminate_flight()
            elif self.check_states(self.action_list_land_here) or self.command_land_here:
                if not self.land_here_activated and \
                   not self.terminate_activated:
                    logger.info('Action: Land here')
                    self.stop_wait_here_timer()
                    self.land_here_activated = True # We can never enter this again
                    self.drone_handler.land_at_current_location()
            elif self.check_states(self.action_list_return_to_launch) or self.command_return_to_launch:
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
               self.check_state(self.gsm_connection_state, action_list) or \
               self.check_state(self.connection_state, action_list))
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