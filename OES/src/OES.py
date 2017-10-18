import pika
import time
import threading
import logging
import json
from GpsModule import GpsModule
from GpsMonitor import GpsMonitor
from DroneHandler import DroneHandler
from OnboardControl import OnboardControl
from Queue import Queue


## Global queues ###
external_gps_queue = Queue()
internal_gps_queue = Queue()
gsm_queue = Queue()

### Classes ###

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


def dummy_signal(signal,source=''):
    print 'Got signal:', signal, 'with source:', source


def dummy_internal_gps():
    return None


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    logger.info('test')
    #external_gps_handler = PikaHandler('externalGps',external_gps_queue)
    #external_gps_handler.start()
    gsm_command_queue = Queue()
    gsm_transmit_queue = Queue()
    drone_handler_signal_queue = Queue()
    external_gps_module = GpsModule('/dev/ttyACM0',9600)
    external_gps_module.start()
    drone_handler = DroneHandler('127.0.0.1:14540',115200,drone_handler_signal_queue)
    onboard_control = OnboardControl(drone_handler, drone_handler_signal_queue, rate = 2)
    gps_monitor = GpsMonitor(onboard_control.signal_gps_state, external_gps_module.get_position, drone_handler.get_position)
    gps_monitor.start()
    onboard_control.start()
    do_exit = False
    while do_exit == False:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True

    external_gps_module.stop()
    gps_monitor.stop()
    onboard_control.stop()
    drone_handler.close()
    print threading.enumerate()

if __name__ == "__main__":
    main()
