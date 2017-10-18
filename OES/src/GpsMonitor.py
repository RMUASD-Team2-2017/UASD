import threading
import time
import utm
import logging
from math import sqrt

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


class GpsMonitor(StoppableThread):

    STATE_OK = 'GPS_OK'
    STATE_TIMEOUT = 'GPS_TIMEOUT'
    STATE_LOST = 'GPS_LOST'
    STATE_LOST_BOTH = 'GPS_LOST_BOTH'
    STATE_MISMATCH = 'GPS_MISMATCH'
    STATE_NO_FIX_YET = 'GPS_NO_FIX_YET'

    TIMEOUT_VALUE = 3
    LOST_TIME_VALUE = 10
    MISMATCH_DISTANCE_ACCEPTANCE_VALUE = 50

    def __init__(self, signal_function, get_external_pos, get_internal_pos, rate = 1):
        StoppableThread.__init__(self)
        self.name = 'GpsMonitor'
        self.signal = signal_function
        self.get_external_pos = get_external_pos
        self.get_internal_pos = get_internal_pos
        self.rate = rate

    def run(self):
        logger.info('GpsMonitor started')
        time.sleep(30) # Allow the gps' to get a fix
        logger.info('GpsMonitor enforced')
        while self.stop_event.is_set() is False:
            external_pos = self.get_external_pos()
            internal_pos = self.get_internal_pos()

            # Check if a fix is obtained
            if external_pos is None or internal_pos is None:
                self.signal(GpsMonitor.STATE_NO_FIX_YET)
                time.sleep(float(1) / float(self.rate))
                continue
            #print json.dumps(external_pos)
            #print json.dumps(internal_pos)
            external_state = self.check_timestamp(external_pos)
            internal_state = self.check_timestamp(internal_pos)

            # Check if GPS is lost or missing for an acceptable amount of seconds (timeout)
            if external_state == GpsMonitor.STATE_LOST or internal_state == GpsMonitor.STATE_LOST:
                source = self.find_source(GpsMonitor.STATE_LOST,external_state,internal_state)
                # If necessary check for source here and send a GpsMonitor.STATE_LOST_BOTH if deemed necessary
                self.signal(GpsMonitor.STATE_LOST,source)
            elif external_state == GpsMonitor.STATE_TIMEOUT or internal_state == GpsMonitor.STATE_TIMEOUT:
                source = self.find_source(GpsMonitor.STATE_TIMEOUT,external_state,internal_state)
                self.signal(GpsMonitor.STATE_TIMEOUT, source)
            else:
                # Check for mismatch
                external_pos_utm = utm.from_latlon(external_pos['position']['lat'], external_pos['position']['lng'])
                internal_pos_utm = utm.from_latlon(internal_pos['position']['lat'], internal_pos['position']['lng'])
                dist = sqrt((external_pos_utm[0]-internal_pos_utm[0])**2 + \
                       (external_pos_utm[1]-internal_pos_utm[1])**2 + \
                       (external_pos['position']['alt'] - internal_pos['position']['alt'])**2)
                if dist <= GpsMonitor.MISMATCH_DISTANCE_ACCEPTANCE_VALUE:
                    self.signal(GpsMonitor.STATE_OK)
                else:
                    self.signal(GpsMonitor.STATE_MISMATCH)

            # Sleep to obtain desired frequency
            time.sleep(float(1)/float(self.rate))
        logger.info('Terminating')

    def check_timestamp(self, position):
        current_time = time.time()
        if current_time - float(position['timestamp']) > float(GpsMonitor.LOST_TIME_VALUE):
            return GpsMonitor.STATE_LOST
        elif current_time - float(position['timestamp']) > float(GpsMonitor.TIMEOUT_VALUE):
            return GpsMonitor.STATE_TIMEOUT
        else:
            return GpsMonitor.STATE_OK

    def find_source(self, state, external_state, internal_state):
        source = 'UNKOWN'
        if external_state == state and internal_state == state:
            source = 'BOTH'
        elif external_state == state:
            source = 'EXTERNAL'
        elif internal_state == state:
            source = 'INTERNAL'
        return source

