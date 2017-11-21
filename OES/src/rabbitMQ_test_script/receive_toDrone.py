#!/usr/bin/env python
import pika

parameters = pika.URLParameters('amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx')
parameters.socket_timeout = 1000
parameters.blocked_connection_timeout = 1000
connection = pika.BlockingConnection(parameters)
channel = connection.channel()


channel.queue_declare(queue='/toDrone')

def callback(ch, method, properties, body):
    print(" [x] Received %r" % body)

channel.basic_consume(callback,
                      queue='/toDrone',
                      no_ack=True)

print(' [*] Waiting for messages. To exit press CTRL+C')
channel.start_consuming()
print(' [*] After waiting')
