import pynmea2
import serial
import threading
import sys
import time
import logging
import pika
import json

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


class GpsModule(StoppableThread):
    def __init__(self,port,baud):
        StoppableThread.__init__(self)
        self.name = 'GpsModule'
        self.port = port
        self.baud = baud
        self.daemon = True
        self.position = None
        self.position_time = None
        self.fix = None
        self.DOP = None
        self.number_of_satellites = None
        self.position_lock = threading.Lock()


    #  GGA          Global Positioning System Fix Data
    #  123519       Fix taken at 12:35:19 UTC
    #  4807.038,N   Latitude 48 deg 07.038' N
    #  01131.000,E  Longitude 11 deg 31.000' E
    #  1            Fix quality: 0 = invalid
    #                            1 = GPS fix (SPS)
    #                            2 = DGPS fix
    #                            3 = PPS fix
	# 		       4 = Real Time Kinematic
	# 		       5 = Float RTK
    #                            6 = estimated (dead reckoning) (2.3 feature)
	# 		       7 = Manual input mode
	# 		       8 = Simulation mode
    #  08           Number of satellites being tracked
    #  0.9          Horizontal dilution of position
    #  545.4,M      Altitude, Meters, above mean sea level
    #  46.9,M       Height of geoid (mean sea level) above WGS84
    #                   ellipsoid
    #  (empty field) time in seconds since last DGPS update
    #  (empty field) DGPS station ID number
    #  *47          the checksum data, always begins with *

    #  GSA      Satellite status
    #  A        Auto selection of 2D or 3D fix (M = manual)
    #  3        3D fix - values include: 1 = no fix
    #                                    2 = 2D fix
    #                                    3 = 3D fix
    #  04,05... PRNs of satellites used for fix (space for 12)
    #  2.5      PDOP (dilution of precision)
    #  1.3      Horizontal dilution of precision (HDOP)
    #  2.1      Vertical dilution of precision (VDOP)
    #  *39      the checksum data, always begins with *

    def run(self):
        # Setup serial connection
        serial_connection = serial.Serial(self.port,self.baud)

        # Setup rabbitmq
        # TODO: Change credentials for production setup
        #credentials = pika.credentials.PlainCredentials('guest', 'guest')
        #connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost',credentials=credentials))
        #channel = connection.channel()
        # Durable: Queue will be stored on disk in case of crash, messages must have delivery mode 2 (persistent)
        #channel.queue_declare(queue='externalGps',durable=True)
        logger.info('GpsModule started')
        while self.stop_event.is_set() is False:
            # Blocking read a line
            line = serial_connection.readline()
            # Remove new line
            line = line.replace('\n','')
            # Check for GGA message
            # GGA - essential fix data which provide 3D location and accuracy data.
            # http://www.gpsinformation.org/dale/nmea.htm#GGA
            nmea = pynmea2.parse(line)

            if nmea.sentence_type == 'GGA':
                if nmea.is_valid:
                    #print nmea
                    with self.position_lock:
                        self.position = {'lat': nmea.latitude, 'lng': nmea.longitude, 'alt': nmea.altitude}
                        self.position_time = time.time()
                        #Publish the position
                        message = {'timestamp': self.position_time, 'position': self.position, 'fix_type': self.fix, 'DOP': self.DOP, 'satellites': self.number_of_satellites}

                    #channel.basic_publish(exchange='', \
                    #    routing_key='externalGps', \
                    #    body=json.dumps(message), \
                    #    properties=pika.BasicProperties( \
                    #    delivery_mode = 2,))

                    logger.debug('GGA ok')
                else:
                    logger.warning('GGA not valid')
                # Do something with the data
            elif nmea.sentence_type ==  'GSA':
                if nmea.is_valid:
                    #print nmea
                    # Count satellites. Each satellites has a PRN in index 2-13
                    # 2-13 are  PRN (a code which each represent one satellite)
                    count = 0
                    # Range(a,b) is [a;b[
                    for i in range(2,14):
                        if str(nmea.data[i]):
                            count += 1
                    with self.position_lock:
                        self.number_of_satellites = count
                        self.fix = int(nmea.data[1])
                        self.DOP = {'PDOP': nmea.data[14],'HDOP': nmea.data[15], 'VDOP': nmea.data[16]}
                        logger.debug('GSA ok')
                        logger.info('GSA - Satellites: %i, Fix: %i', self.number_of_satellites, self.fix)
                else:
                    logger.warning('GSA not valid')

        logger.info('GpsModule terminating')
        #connection.close()

    def get_position(self):
        with self.position_lock:
            return {'timestamp': self.position_time, 'position': self.position, 'fix_type': self.fix, 'DOP': self.DOP, 'satellites': self.number_of_satellites}


def main():
    gps = GpsModule(sys.argv[1],sys.argv[2])
    gps.start()
    do_exit = False
    while do_exit == False:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            # Ctrl+C was hit - exit program
            do_exit = True
    gps.stop()

if __name__ == "__main__":
    main()
