#!/usr/bin/env python
import rospy
import pika
import json
import threading
import time
from gcs.msg import toDroneData
from Queue import Queue
from std_msgs.msg import String


class GsmTalker:

    def __init__(self, pika_connection_string, topic="/toDrone", heartbeat_rate=1.0):
        self.send_to_drone_subscriber = rospy.Subscriber("gsm_talker/toDrone", toDroneData,
                                                         self.send_to_drone_subscriber_callback, queue_size=10)
        self.request_id_subscriber = rospy.Subscriber("/web_interface/request_id", String, self.requestIdCallback,
                                                                          queue_size=1)
        self.topic = topic
        parameters = pika.URLParameters(pika_connection_string)
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()
        self.channel.queue_declare(queue=self.topic)
        self.transmit_queue = Queue()
        self.heartbeat_timer = None
        self.heartbeat_rate = heartbeat_rate
        self.request_id = '0'

    def send_to_drone_subscriber_callback(self,data):
        msg = {'type': data.type,
               'value': data.value}
        self.transmit_queue.put(msg)

    def requestIdCallback(self, data):
        self.request_id = data.data


    def run(self):
        while not self.transmit_queue.empty():
            self.publish_json(self.transmit_queue.get())

    def publish(self, message):
        self.channel.basic_publish(exchange='', \
                                   routing_key=self.topic, \
                                   body=message, \
                                   properties=pika.BasicProperties( \
                                       delivery_mode=1, ))  # Non-persistent

    def publish_json(self, message):
        self.publish(json.dumps(message))

    def heartbeat(self):
        # Start a timer to call heartbeat again after heartbeat_rate seconds
        # Referenced in self to enable stopping upon thread exit
        # The idea is inspired by group 1
        self.heartbeat_timer = threading.Timer(self.heartbeat_rate, self.heartbeat)
        self.heartbeat_timer.start()

        # Publish the message
        msg = {'type': 'HEARTBEAT', 'value': time.time(), 'uuid': self.request_id}
        self.transmit_queue.put(msg)

    def start_heartbeat(self):
        self.heartbeat()

    def stop_heartbeat(self):
        if self.heartbeat_timer.is_alive():
            self.heartbeat_timer.cancel()

    def cleanup(self):
        self.stop_heartbeat()
        self.channel.close()


def main():
    rospy.init_node('gsm_talker')
    rospy.loginfo('Started')
    pika_connection_string = 'amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx'
    gsmTalker = GsmTalker(pika_connection_string)
    gsmTalker.start_heartbeat()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        gsmTalker.run()
        rate.sleep()

    gsmTalker.cleanup()
    rospy.loginfo('Terminated')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #Catches exceptions to e.g. shutdown
        print 'Got exception:'
        pass
