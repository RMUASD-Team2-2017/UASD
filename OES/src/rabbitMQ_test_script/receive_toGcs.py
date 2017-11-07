#!/usr/bin/env python
import pika

parameters = pika.URLParameters('amqp://wollgvkx:6NgqFYICcYPdN08nHpQMktCoNS2yf2Z7@lark.rmq.cloudamqp.com/wollgvkx')
connection = pika.BlockingConnection(parameters)
channel = connection.channel()


channel.queue_declare(queue='/toGcs')

def callback(ch, method, properties, body):
    print(" [x] Received %r" % body)

channel.basic_consume(callback,
                      queue='/toGcs',
                      no_ack=True)

print(' [*] Waiting for messages. To exit press CTRL+C')
channel.start_consuming()
print(' [*] After waiting')
