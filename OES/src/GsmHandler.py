import threading
import pika
import logging
import json
import time
from Queue import Queue
from ConnectionMonitor import ConnectionMonitor
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


class GsmReceiver(StoppableThread):
    def __init__(self,pika_connection_string, command_queue, topic='/toDrone'):
        StoppableThread.__init__(self)
        self.daemon = False
        self.topic = topic
        self.command_queue = command_queue
        self.heartbeat = None
        self.heartbeat_lock = threading.Lock()

        # Pika setup
        parameters = pika.URLParameters(pika_connection_string)
        parameters.socket_timeout = 1000
        parameters.blocked_connection_timeout = 1000
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()
        # Connect with the queue
        self.channel.queue_declare(queue=self.topic)
        # Only one unacknowledged message allowed at a time
        self.channel.basic_qos(prefetch_count=1)

    def run(self):
        logger.info('Started')

        ### Consume ###
        # The consuming is based on: https://stackoverflow.com/questions/32220057/interrupt-thread-with-start-consuming-method-of-pika
        for message in self.channel.consume(self.topic,inactivity_timeout=1):
            if self.stop_event.is_set() == True:
                # Exit the loop
                # The channel must be closed to be able to change settings on restart
                self.channel.cancel()
                break
            if not message:
                continue
            # We have a message if we get here
            method, properties, body = message
            self.channel.basic_ack(method.delivery_tag)
            decoded = json.loads(body)
            if decoded['type'] == 'HEARTBEAT':
                with self.heartbeat_lock:
                    self.heartbeat = time.time()
            else:
                self.command_queue.put(decoded)
        self.channel.close()
        logger.info('Terminating')

    def get_heartbeat(self):
        with self.heartbeat_lock:
            return self.heartbeat


class GsmTalker(StoppableThread):
    def __init__(self, pika_connection_string, transmit_queue, topic='/toGcs', heartbeat_rate=1.0):
        StoppableThread.__init__(self)
        self.topic = topic
        self.transmit_queue = transmit_queue
        self.heartbeat_rate = heartbeat_rate
        self.heartbeat_timer = None
        self.connection_state = ConnectionMonitor.STATE_CON_TELEMETRY_LOST_GSM_LOST
        self.gps_state = GpsMonitor.STATE_LOST

        parameters = pika.URLParameters(pika_connection_string)
        #parameters.socket_timeout = 1000
        #parameters.blocked_connection_timeout = 1000
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()
        self.channel.queue_declare(queue=self.topic)

    def run(self):
    	
        # Start transmitting a heartbeat
        self.heartbeat()
        # Publish when messages are queued
        while self.stop_event.is_set() is False:
            time.sleep(0.1)
            while not self.transmit_queue.empty():
                msg = self.transmit_queue.get()
                if msg['type'] == 'CONNECTION_STATE':
                    self.connection_state = msg['value']
                elif msg['type'] == 'GPS_STATE':
                    self.gps_state = msg['value']
                else:
                    self.publish_json(msg)

        if self.heartbeat_timer.is_alive():
            self.heartbeat_timer.cancel()
        self.channel.close()

    def publish(self,message):
        self.channel.basic_publish(exchange='', \
                                   routing_key=self.topic, \
                                   body=message, \
                                   properties=pika.BasicProperties( \
                                    delivery_mode=1,)) # Non-persistent

    def publish_json(self,message):
        self.publish(json.dumps(message))

    def heartbeat(self):
        # Start a timer to call heartbeat again after heartbeat_rate seconds
        # Referenced in self to enable stopping upon thread exit
        # The idea is inspired by group 1
        self.heartbeat_timer = threading.Timer(self.heartbeat_rate, self.heartbeat)
        self.heartbeat_timer.start()

        # Publish the message
        msg = {'type': 'HEARTBEAT', 'timestamp': time.time(), 'connectionState': self.connection_state, 'gpsState': self.gps_state}
        self.transmit_queue.put(msg)


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    logger.info('test')
    gsm_command_queue = Queue()
    gsm_transmit_queue = Queue()
    # Pika is not thread safe so we just pass the connection string and not a connection
    pika_connection_string = 'amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx'
    receiver = GsmReceiver(pika_connection_string,gsm_command_queue)
    receiver.start()
    talker = GsmTalker(pika_connection_string,gsm_transmit_queue)
    talker.start()

    connection_msg = {'type': 'CONNECTION_STATE', 'value': 1}
    gsm_transmit_queue.put(connection_msg)

    gps_msg = {'type': 'GPS_STATE', 'value': 2}
    gsm_transmit_queue.put(gps_msg)

    do_exit = False
    while do_exit == False:
        try:
            while not gsm_command_queue.empty():
                print gsm_command_queue.get()
                time.sleep(0.1)
        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True

    receiver.stop()
    talker.stop()


if __name__ == "__main__":
    main()
