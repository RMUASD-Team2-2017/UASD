import pika
import time
import threading
import logging
import json
import ConfigParser
import os
from GpsModule import GpsModule
from GpsMonitor import GpsMonitor
from DroneHandler import DroneHandler, DroneHandler_pymavlink
from OnboardControl import OnboardControl
from ConnectionMonitor import ConnectionMonitor, MavlinkListener
from GsmHandler import GsmReceiver, GsmTalker
from Queue import Queue
from WebInterface import WebInterface

## Settings ##

dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path = dir_path + "/" + 'config.cfg'
print dir_path
config = ConfigParser.RawConfigParser()
config.read(dir_path)
fc_connection_string = config.get('drone_connection','fc_connection_string')
fc_connection_baud = int(config.get('drone_connection','fc_connection_baud'))
to_drone_sniff_port = config.get('drone_connection','sniff_connection')
to_drone_sniff_baud = int(config.get('drone_connection','sniff_baud'))
gps_string = config.get('drone_connection','gps_string')
gps_baud = config.get('drone_connection','gps_baud')
geofence_file = config.get('drone_connection','geofence_file')
pika_connection_string = config.get('drone_connection','pika_connection_string')
print '### Settings ####'
print '*** Config file:', dir_path, '***'
print
print fc_connection_string
print fc_connection_baud
print to_drone_sniff_port
print to_drone_sniff_baud
print gps_string
print gps_baud
print geofence_file
print pika_connection_string
print

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
    command_queue = Queue()
    gsm_transmit_queue = Queue()
    drone_handler_signal_queue = Queue()
    request_id_queue = Queue()
    external_gps_module = GpsModule(gps_string,gps_baud)
    external_gps_module.start()
    drone_handler = DroneHandler_pymavlink(fc_connection_string,drone_handler_signal_queue,fc_connection_baud) # NOTE: The bad must be implemented in instantiation of dronekit
    drone_handler.start()
    onboard_control = OnboardControl(drone_handler, drone_handler_signal_queue, gsm_transmit_queue, command_queue, rate = 1)
    gps_monitor = GpsMonitor(onboard_control.signal_gps_state, external_gps_module.get_position, drone_handler.get_position, drone_handler.get_home_position,
                             geofencefile=geofence_file)
    gps_monitor.start()

    # Web interface
    web_interface = WebInterface(command_queue, request_id_queue)
    web_interface.start()

    # GsmHandler
    #pika_connection_string = 'amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx'
    gsm_listener = GsmReceiver(pika_connection_string,command_queue, web_interface.get_last_ping_time)
    gsm_listener.start()
    gsm_talker = GsmTalker(pika_connection_string,gsm_transmit_queue, web_interface.get_last_ping_time)
    gsm_talker.start()

    # To drone sniffer
    sniff_to_drone = MavlinkListener(to_drone_sniff_port,baud=57600)
    sniff_to_drone.start()

    # ConnectionMonitor
    #connection_monitor = ConnectionMonitor(onboard_control.signal_connection_state, sniff_to_drone.get_heartbeat,
    #                                       gsm_listener.get_heartbeat)

    connection_monitor = ConnectionMonitor(onboard_control.signal_connection_state, sniff_to_drone.get_heartbeat,
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
    drone_handler.stop()
    gsm_listener.stop()
    #gsm_talker.stop()
    #sniff_to_drone.stop()
    connection_monitor.stop()
    web_interface.stop()
    print threading.enumerate()
    logger.info('OES terminated')

if __name__ == "__main__":
    main()
