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


### Global queues ####
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


class PikaHandler(StoppableThread):
    def __init__(self,pika_queue, internal_queue):
        StoppableThread.__init__(self)
        self.daemon = False
        self.pika_queue = pika_queue
        self.internal_queue = internal_queue
    def run(self):
        ### Setup rabbitmq ###
        # TODO: Change credentials for production version
        credentials = pika.credentials.PlainCredentials('guest','guest')
        connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost',credentials=credentials))
        channel = connection.channel()
        # Connect with the queue
        channel.queue_declare(queue=self.pika_queue, durable=True)
        # Only one unacknowledged message allow at a time
        channel.basic_qos(prefetch_count=1)
        # The line below are used when the callback is used
        # channel.basic_consume(self.pika_callback, queue='externalGps')

        logging.info('Pika ready to receive')

        ### Consume ###
        # The consuming is based on: https://stackoverflow.com/questions/32220057/interrupt-thread-with-start-consuming-method-of-pika
        for message in channel.consume(self.pika_queue,inactivity_timeout=1):
            if self.stop_event.is_set() == True:
                # Exit the loop
                # The channel must be closed to be able to change settings on restart
                channel.cancel()
                break
            if not message:
                print 'No message'
                continue
            # We have a message if we get here
            method, properties, body = message
            channel.basic_ack(method.delivery_tag)
            self.internal_queue.put(json.loads(body))
            print(method)

        # The lines below are used when the callback is used
        #while ( self.stop_event.is_set() == False ):
        #    channel.start_consuming()

    def pika_callback(self,ch, method, properties, body):
        print(" [x] Received %r" % body)
        ch.basic_ack(delivery_tag = method.delivery_tag)



class ControlLogic:
    ### State variables ###
    # gps_state
    GPS_OK = 0
    GPS_MISMATCH = 1
    GPS_TIMEOUT = 2
    GPS_FIX_LOST = 3

    # Values in seconds
    FIX_LOST_TIMEOUT = 10

    def __init__(self):
        self.gps_state = GPS_OK
        self.external_gps_position = None
        self.internal_gps_position = None

    def gps_monitor(self):
        # Update position
        updated = 0
        while not external_gps_queue.empty:
            self.external_gps_position = external_gps_queue.get()
            updated += 1
        while not internal_gps_queue.empty:
            self.internal_gps_position = internal_gps_queue.get()
            updated += 1

        # If position is updated, calculate distance
        if updated > 0:
            pass

        positions = [self.external_gps_position,self.internal_gps_position]
        current_time = time.time()
        for position in positions:
            if current_time-self.external_gps_position['timestamp'] > ControlLogic.FIX_LOST_TIMEOUT:
                # Signal land at current location
                continue


    def gsm_monitor(self):
        pass
    def controller(self):
        pass

def dummy_signal(signal,source):
    print 'Got signal:', signal, 'with source:', source

def dummy_signal(signal):
    print 'Got signal:', signal

def dummy_internal_gps():
    return None

def main():
    logging.getLogger('test').setLevel('INFO')
    #external_gps_handler = PikaHandler('externalGps',external_gps_queue)
    #external_gps_handler.start()
    external_gps_module = GpsModule('/dev/ttyACM0',9600)
    external_gps_module.start()
    drone_handler = DroneHandler('127.0.0.1:14540',115200,dummy_signal)
    gps_monitor = GpsMonitor(dummy_signal,external_gps_module.get_position,drone_handler.get_position)
    gps_monitor.start()
    onboard_control = OnboardControl(drone_handler,rate = 2)
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

if __name__ == "__main__":
    main()
