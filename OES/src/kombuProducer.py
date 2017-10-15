import kombu
import Queue

class kombuProducer:
    def __init__(self,exchange,topic):
        self.connection = kombu.Connection('amqp://guest:guest@localhost:5672//')
        self.producer = self.connection.Producer()
        self.queue = Queue.Queue()
        self.topic

    def publish(self,data,topic):
        self.queue.put(data)


    def publisher(self)
        while True:
            if not self.queue.empty()
