import threading
import time
from GpsMonitor import GpsMonitor

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
    def __init__(self,drone_handler,rate = 1):
        StoppableThread.__init__(self)
        self.rate = float(rate)
        self.drone_handler = drone_handler
        self.signal_gps_lock = threading.Lock()
        self.signal_drone_heartbeat_lock = threading.Lock()
        self.signal_gsm_lock = threading.Lock()
        self.gps_state = None
        self.gps_source = None
        self.drone_heartbeat = None
        self.drone_connection_state = None

        self.gsm_connection_state = None

        self.action_list_terminate = []
        self.action_list_land_here = [GpsMonitor.STATE_LOST]
        self.action_list_return_to_launch = [OnboardControl.CONNECTION_LOST]
        self.action_list_wait_here = [GpsMonitor.STATE_TIMEOUT,
                                      GpsMonitor.STATE_MISMATCH]
        self.action_list_no_action = [GpsMonitor.STATE_OK,
                                      GpsMonitor.STATE_NO_FIX_YET,
                                      OnboardControl.CONNECTION_OK]

        self.timer_wait_here = threading.Timer(10.0,self.disable_wait_here)

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
        terminate_activated = False
        land_here_activated = False
        return_to_launch_activated = False
        wait_here_activated = False
        while self.stop_event.is_set() is False:

            # Monitor heartbeat from dronekit
            with self.signal_drone_heartbeat_lock:
                if time.time() - self.drone_heartbeat > OnboardControl.HEARTBEAT_LOST_TIME:
                    self.drone_connection_state = OnboardControl.CONNECTION_LOST
                else:
                    self.drone_connection_state = OnboardControl.CONNECTION_OK

            # Severity order: Terminate, land here, return to launch, wait 10 seconds
            if self.check_states(self.action_list_terminate):
                if not terminate_activated:
                    terminate_activated = True # We can never enter this again
                    self.drone_handler.terminate_flight()
            elif self.check_states(self.action_list_land_here):
                if not land_here_activated:
                    land_here_activated = True # We can never enter this again
                    self.drone_handler.land_at_current_location()
            elif self.check_states(self.action_list_return_to_launch):
                if not return_to_launch_activated:
                    return_to_launch_activated = True # We can never enter this again
                    self.drone_handler.return_to_launch()
            elif self.check_states(self.action_list_wait_here):
                if not wait_here_activated:
                    wait_here_activated = True
                    self.drone_handler.loiter()
                    self.timer_wait_here.start()

            elif self.check_states(self.action_list_no_action):
                pass  # No-op

            # Sleep to obtain desired rate
            time.sleep(1.0/self.rate)

    def check_state(self,state,action_list):
        return state in action_list

    def check_states(self,action_list):
        ret = (self.check_state(self.gps_state,action_list) and \
               self.check_state(self.drone_connection_state, action_list) and \
               self.check_state(self.gsm_connection_state, action_list))
        return ret

    def disable_wait_here(self):
        self.drone_handler.resume_mission()