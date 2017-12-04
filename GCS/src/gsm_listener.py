#!/usr/bin/env python
import rospy
import pika
import json
import time
import threading
from std_msgs.msg import String
from std_msgs.msg import Bool
from Queue import Queue


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


class GsmListener(StoppableThread):
    HEARTBEAT_TIMEOUT = 5.0

    def __init__(self, pika_connection_string, topic='/toGcs'):
        StoppableThread.__init__(self)
        self.topic = topic
        self.last_heartbeat = None
        self.receive_queue = Queue()

        self.heartbeat_lock = threading.Lock()

        parameters = pika.URLParameters(pika_connection_string)
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()
        # Connect with the queue
        self.channel.queue_declare(queue=self.topic)
        # Only one unacknowledged message allowed at a time
        self.channel.basic_qos(prefetch_count=1)

        self.from_drone_publisher = rospy.Publisher("/gsm_listener/fromDrone", String, queue_size=10)
        self.gsm_heartbeat_publisher = rospy.Publisher("/gsm_listener/heartbeat", Bool, queue_size=1)

    def run(self):
        for message in self.channel.consume(self.topic, inactivity_timeout=1):
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
                #print 'Received Heartbeat:',decoded['connectionState'],decoded['gpsState']
                with self.heartbeat_lock:
                    self.last_heartbeat = time.time()
                self.receive_queue.put(decoded)
            else:
                self.receive_queue.put(decoded)
        self.channel.close()

    def handle_receive_queue(self):
        #rospy.loginfo('handler')
        while not self.receive_queue.empty():
            #rospy.loginfo('received')
            msg = self.receive_queue.get()
            #print json.dumps(msg)
            self.from_drone_publisher.publish(json.dumps(msg))

    def check_heartbeat(self):
        with self.heartbeat_lock:
            if self.last_heartbeat is None:
                self.gsm_heartbeat_publisher.publish(False)
            elif time.time() - self.last_heartbeat > GsmListener.HEARTBEAT_TIMEOUT:
                self.gsm_heartbeat_publisher.publish(False)
            else:
                self.gsm_heartbeat_publisher.publish(True)


def main():
    rospy.init_node('gsm_talker')
    rospy.loginfo('Started')
    pika_connection_string = 'amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx'
    gsmListener = GsmListener(pika_connection_string)
    gsmListener.start()

    rate = rospy.Rate(20)
    heartbeat_check_counter = 1
    while not rospy.is_shutdown():
        if heartbeat_check_counter == 1:
            gsmListener.check_heartbeat()
        heartbeat_check_counter += 1
        heartbeat_check_counter %= 20
        gsmListener.handle_receive_queue()
        rate.sleep()

    gsmListener.stop()
    rospy.loginfo('Terminated')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #Catches exceptions to e.g. shutdown
        print 'Got exception:'
        pass
