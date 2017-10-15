import pynmea2
import serial
import threading
import sys
import time
import Queue

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

class gpsModule(StoppableThread):
    def __init__(self,port,baud):
        StoppableThread.__init__(self)
        self.port = port
        self.baud = baud
        self.daemon = True

    def run(self):
        #connection = serial.Serial(self.port,self.baud)
        while ( self.stop_event.is_set() == False ):
            # If data is available
            if(connection.inWaiting()>0):
                # Read a line and remove new line
                line = connection.readline()
                line = line.replace('\n','')
                # Check for GGA message
                # GGA - essential fix data which provide 3D location and accuracy data.
                # http://www.gpsinformation.org/dale/nmea.htm#GGA
                if 'GGA' in line:
                    nmeaString = pynmea2.parse(line)
                    # Do something with the data
        print 'Exiting loop'


def main():
    gps = gpsModule(sys.argv[1],sys.argv[2])
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
