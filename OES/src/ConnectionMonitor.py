import threading
import time
import logging
from pymavlink import mavutil

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


class ConnectionMonitor(StoppableThread):
    # Internal states
    STATE_CON_NOT_YET = 'STATE_NO_CON_YET'
    STATE_CON_OK = 'STATE_CON_OK'
    STATE_CON_TIMEOUT = 'STATE_CON_TIMEOUT'
    STATE_CON_LOST = 'STATE_CON_LOST'

    # External states
    # STATE_CON_OK ALSO USED HERE
    STATE_CON_TELEMETRY_OK_GSM_TIMEOUT = 'STATE_CON_TLM_OK_GSM_T'
    STATE_CON_TELEMETRY_OK_GSM_LOST = 'STATE_CON_TLM_OK_GSM_L'

    STATE_CON_TELEMETRY_TIMEOUT_GSM_OK = 'STATE_CON_TLM_T_GSM_OK'
    STATE_CON_TELEMETRY_TIMEOUT_GSM_TIMEOUT = 'STATE_CON_TLM_T_GSM_T'
    STATE_CON_TELEMETRY_TIMEOUT_GSM_LOST = 'STATE_CON_TLM_T_GSM_L'

    STATE_CON_TELEMETRY_LOST_GSM_OK = 'STATE_CON_TLM_L_GSM_OK'
    STATE_CON_TELEMETRY_LOST_GSM_TIMEOUT = 'STATE_CON_TLM_L_GSM_T'
    STATE_CON_TELEMETRY_LOST_GSM_LOST = 'STATE_CON_TLM_L_GSM_L'

    # Values
    HEARTBEAT_TIMEOUT = 5.0
    HEARTBEAT_LOST = 10.0

    def __init__(self, signal_function, get_hb_to_drone_sniff,
                 get_hb_gsm, get_hb_drone_serial2=None,
                 get_hb_from_drone_sniff=None, rate=1):
        StoppableThread.__init__(self)
        self.name = 'ConnectionMonitor'
        self.signal = signal_function
        self.get_heartbeat_to_drone = get_hb_to_drone_sniff
        self.get_heartbeat_from_drone = get_hb_from_drone_sniff
        self.get_heartbeat_drone_serial2 = get_hb_drone_serial2
        self.get_heartbeat_gsm = get_hb_gsm
        self.rate = rate

    def run(self):
        logger.info('Started')
        time.sleep(30) # Allow the connections to start and connect
        logger.info('Enforced')
        while self.stop_event.is_set() is False:
            # Get the heartbeats
            heartbeat_to_drone = self.get_heartbeat_to_drone()
            #heartbeat_from_drone = self.get_heartbeat_from_drone()
            gsm_heartbeat = self.get_heartbeat_gsm()
            #drone_serial2_heartbeat = self.get_heartbeat_drone_serial2()

            # Check them
            to_drone_state = self.check_heartbeat(heartbeat_to_drone)
            gsm_state = self.check_heartbeat(gsm_heartbeat)
            # Don't monitor connection to ground. GC is responsible of that.
            # from_drone_state = self.check_heartbeat(heartbeat_from_drone)

            # We don't monitor this for now as it is simply a wire connection.
            # The fc will probably be broke if this is not working
            # If we wan't to monitor it we should signal to GCS such that it can terminate
            # drone_serial2_state = self.check_heartbeat(drone_serial2_heartbeat)

            # Cases
            if gsm_state == ConnectionMonitor.STATE_CON_NOT_YET or \
               to_drone_state == ConnectionMonitor.STATE_CON_NOT_YET:
                self.signal(ConnectionMonitor.STATE_CON_NOT_YET)
                logger.info('Not connected yet')
            elif to_drone_state == ConnectionMonitor.STATE_CON_OK:
                if gsm_state == ConnectionMonitor.STATE_CON_OK:
                    self.signal(ConnectionMonitor.STATE_CON_OK)
                    logger.debug(ConnectionMonitor.STATE_CON_OK)
                elif gsm_state == ConnectionMonitor.STATE_CON_TIMEOUT:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_OK_GSM_TIMEOUT)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_OK_GSM_TIMEOUT)
                elif gsm_state == ConnectionMonitor.STATE_CON_LOST:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_OK_GSM_LOST)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_OK_GSM_LOST)
                else:
                    logger.warning('Telemetry state ok. GSM wrong state')
            elif to_drone_state == ConnectionMonitor.STATE_CON_TIMEOUT:
                if gsm_state == ConnectionMonitor.STATE_CON_OK:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_OK)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_OK)
                elif gsm_state == ConnectionMonitor.STATE_CON_TIMEOUT:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_TIMEOUT)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_TIMEOUT)
                elif gsm_state == ConnectionMonitor.STATE_CON_LOST:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_LOST)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_TIMEOUT_GSM_LOST)
                else:
                    logger.warning('Telemetry state TIMEOUT. GSM wrong state')
            elif to_drone_state == ConnectionMonitor.STATE_CON_LOST:
                if gsm_state == ConnectionMonitor.STATE_CON_OK:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_OK)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_OK)
                elif gsm_state == ConnectionMonitor.STATE_CON_TIMEOUT:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_TIMEOUT)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_TIMEOUT)
                elif gsm_state == ConnectionMonitor.STATE_CON_LOST:
                    self.signal(ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_LOST)
                    logger.debug(ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_LOST)
                else:
                    logger.warning('Telemetry state LOST. GSM wrong state')
            else:
                logger.warning('Telemetry wrong state')

            # Sleep to obtain desired frequency (rate in Hz)
            time.sleep(float(1) / float(self.rate))
        logger.info('Terminating')

    def check_heartbeat(self,heartbeat):
        if heartbeat is None:
            return ConnectionMonitor.STATE_CON_NOT_YET
        else:
            current_time = time.time()
            if current_time - heartbeat < ConnectionMonitor.HEARTBEAT_TIMEOUT:
                return ConnectionMonitor.STATE_CON_OK
            elif current_time -heartbeat < ConnectionMonitor.HEARTBEAT_LOST:
                return ConnectionMonitor.STATE_CON_TIMEOUT
            else:
                return ConnectionMonitor.STATE_CON_LOST


class MavlinkListener(StoppableThread):
    def __init__(self,port,baud=None):
        StoppableThread.__init__(self)
        self.heartbeat = None
        self.heartbeat_lock = threading.Lock()
        self.mav_interface = None
        if baud is None:
            self.mav_interface = mavutil.mavlink_connection(port, autoreconnect=True)
        else:
            self.mav_interface = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)

    def run(self):
        while self.stop_event.is_set() is False:
            m = self.mav_interface.recv_msg()
            if m:
                if (m.get_type() == 'HEARTBEAT'):
                    logger.warning('Heartbeat')
                    with self.heartbeat_lock:
                        self.heartbeat = time.time()

            # Avoid busy loop
            time.sleep(0.01)

    def get_heartbeat(self):
        with self.heartbeat_lock:
            return self.heartbeat
