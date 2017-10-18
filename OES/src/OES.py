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
