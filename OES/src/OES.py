import pika
import time
import threading
import logging
import json
from GpsModule import GpsModule
from GpsMonitor import GpsMonitor
from DroneHandler import DroneHandler
from OnboardControl import OnboardControl
from ConnectionMonitor import ConnectionMonitor, MavlinkListener
from GsmHandler import GsmReceiver, GsmTalker
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

def dummy_get_heartbeat():
    return time.time()

def main():
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('OES started')
    gsm_command_queue = Queue()
    gsm_transmit_queue = Queue()
    drone_handler_signal_queue = Queue()
    external_gps_module = GpsModule('/dev/ttyACM0',9600)
    external_gps_module.start()
    drone_handler = DroneHandler('127.0.0.1:14540',115200,drone_handler_signal_queue)
    onboard_control = OnboardControl(drone_handler, drone_handler_signal_queue, gsm_transmit_queue, gsm_command_queue, rate = 1)
    gps_monitor = GpsMonitor(onboard_control.signal_gps_state, external_gps_module.get_position, drone_handler.get_position,
                             geofencefile="/home/mathias/Dropbox/ROBTEK/9.-semester/RMUASD/UASD_share/geofence/geofence.txt")
    gps_monitor.start()

    # GsmHandler
    pika_connection_string = 'amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx'
    gsm_listener = GsmReceiver(pika_connection_string,gsm_command_queue)
    gsm_listener.start()
    #gsm_talker = GsmTalker(pika_connection_string,gsm_transmit_queue)
    #gsm_talker.start()

    # To drone sniffer
    #to_drone_sniff_port = '/dev/ttyUSB0'
    #sniff_to_drone = MavlinkListener(to_drone_sniff_port,baud=57600)
    #sniff_to_drone.start()

    # ConnectionMonitor
    #connection_monitor = ConnectionMonitor(onboard_control.signal_connection_state, sniff_to_drone.get_heartbeat,
    #                                       gsm_listener.get_heartbeat)

    connection_monitor = ConnectionMonitor(onboard_control.signal_connection_state, dummy_get_heartbeat,
                                           gsm_listener.get_heartbeat, get_hb_drone_serial2=drone_handler.get_heartbeat)

    connection_monitor.start()

    # Start the control
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
    gsm_listener.stop()
    #gsm_talker.stop()
    #sniff_to_drone.stop()
    connection_monitor.stop()
    print threading.enumerate()
    logger.info('OES terminated')

if __name__ == "__main__":
    main()
