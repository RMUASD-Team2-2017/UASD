#!/usr/bin/env python
import pika
import json
parameters = pika.URLParameters('amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx')
#connection = pika.BlockingConnection(pika.ConnectionParameters(
#        host='amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx'))

connection = pika.BlockingConnection(parameters)

channel = connection.channel()


channel.queue_declare(queue='/toGcs')
msg = {'type': 'HEARTBEAT'}
msg_encoded = json.dumps(msg)
channel.basic_publish(exchange='',
                      routing_key='/toGcs',
                      body=msg_encoded)
print(" [x] Sent msg")
connection.close()
