import threading
import time
import utm
import logging
from math import sqrt

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


class GpsMonitor(StoppableThread):

    STATE_OK = 'GPS_OK'
    STATE_TIMEOUT = 'GPS_TIMEOUT'
    STATE_LOST = 'GPS_LOST'
    STATE_LOST_BOTH = 'GPS_LOST_BOTH'
    STATE_MISMATCH = 'GPS_MISMATCH'
    STATE_NO_FIX_YET = 'GPS_NO_FIX_YET'
    STATE_GEOFENCE_BREACH = 'GPS_GEOFENCE_BREACH'

    TIMEOUT_VALUE = 3
    LOST_TIME_VALUE = 10
    MISMATCH_DISTANCE_ACCEPTANCE_VALUE = 50

    def __init__(self, signal_function, get_external_pos, get_internal_pos, geofencefile = None, rate = 1):
        StoppableThread.__init__(self)
        self.name = 'GpsMonitor'
        self.signal = signal_function
        self.get_external_pos = get_external_pos
        self.get_internal_pos = get_internal_pos
        self.rate = rate
        self.geofence = None
        if geofencefile is not None:
            self.geofence = Geofence(geofencefile)

    def run(self):
        logger.info('GpsMonitor started')
        time.sleep(30) # Allow the gps' to get a fix
        logger.info('GpsMonitor enforced')
        while self.stop_event.is_set() is False:
            external_pos = self.get_external_pos()
            internal_pos = self.get_internal_pos()

            # Check if a fix is obtained
            # We don't execute the checks below before we have a fix.
            # NOTE: If we get here in flight we're in trouble as we don't check the states when we get in here
            if external_pos is None or internal_pos is None or \
               external_pos['timestamp'] == None \
               or internal_pos['timestamp'] == None:
                self.signal(GpsMonitor.STATE_NO_FIX_YET)
                time.sleep(float(1) / float(self.rate))
                continue
            #print json.dumps(external_pos)
            #print json.dumps(internal_pos)

            external_state = self.check_timestamp(external_pos)
            internal_state = self.check_timestamp(internal_pos)

            # Check if GPS is lost or missing for an acceptable amount of seconds (timeout)
            if external_state == GpsMonitor.STATE_LOST or internal_state == GpsMonitor.STATE_LOST:
                source = self.find_source(GpsMonitor.STATE_LOST,external_state,internal_state)
                # If necessary check for source here and send a GpsMonitor.STATE_LOST_BOTH if deemed necessary
                self.signal(GpsMonitor.STATE_LOST,source)
            elif external_state == GpsMonitor.STATE_TIMEOUT or internal_state == GpsMonitor.STATE_TIMEOUT:
                source = self.find_source(GpsMonitor.STATE_TIMEOUT,external_state,internal_state)
                self.signal(GpsMonitor.STATE_TIMEOUT, source)
            else:
                # Check for mismatch
                external_pos_utm = utm.from_latlon(external_pos['position']['lat'], external_pos['position']['lng'])
                internal_pos_utm = utm.from_latlon(internal_pos['position']['lat'], internal_pos['position']['lng'])
                dist = sqrt((external_pos_utm[0]-internal_pos_utm[0])**2 + \
                       (external_pos_utm[1]-internal_pos_utm[1])**2 + \
                       (external_pos['position']['alt'] - internal_pos['position']['alt'])**2)
                if dist <= GpsMonitor.MISMATCH_DISTANCE_ACCEPTANCE_VALUE:
                    if not self.geofence is None:
                        if self.geofence.is_point_legal((internal_pos_utm[0], internal_pos_utm[1], internal_pos['position']['alt'])):
                            self.signal(GpsMonitor.STATE_OK)
                        else:
                            self.signal(GpsMonitor.STATE_GEOFENCE_BREACH)
                else:
                    self.signal(GpsMonitor.STATE_MISMATCH)

            # Sleep to obtain desired frequency
            time.sleep(float(1)/float(self.rate))
        logger.info('Terminating')

    def check_timestamp(self, position):
        current_time = time.time()
        if current_time - float(position['timestamp']) > float(GpsMonitor.LOST_TIME_VALUE):
            return GpsMonitor.STATE_LOST
        elif current_time - float(position['timestamp']) > float(GpsMonitor.TIMEOUT_VALUE):
            return GpsMonitor.STATE_TIMEOUT
        else:
            return GpsMonitor.STATE_OK

    def find_source(self, state, external_state, internal_state):
        source = 'UNKOWN'
        if external_state == state and internal_state == state:
            source = 'BOTH'
        elif external_state == state:
            source = 'EXTERNAL'
        elif internal_state == state:
            source = 'INTERNAL'
        return source


class Geofence:
    def __init__(self, fence, convert_to_utm=True, obstacles=None, maxHeight=100.0):
        self.convert_to_utm = convert_to_utm
        self.fence = self.load_from_file(fence)
        self.obstacles = obstacles
        self.maxHeight = float(maxHeight)

    def load_from_file(self,file):
        count = 0
        points = []
        f = open(file,"r")
        for line in f:
            count = count +1
            # Remove new lines and split in fields
            line = line.replace('\n', '')
            fields = line.split(',')
            x = float(fields[0])
            y = float(fields[1])
            if self.convert_to_utm:
                utm_point = utm.from_latlon(x,y)
                x = utm_point[0]
                y = utm_point[1]
            points.append((x, y))

        logger.info('Loaded geofence with %i points', count)

        return points

    def is_point_legal(self, point3d):
        x = point3d[0]
        y = point3d[1]
        altitude = point3d[2]

        # Are we above maximal height
        if altitude > self.maxHeight:
            return False

        # Are we inside the fence
        point2d = (x, y)
        if self.inside_fence(point2d):
            return True

        return False

    def inside_fence(self, point2d):
        return bool(self.cn_PnPoly(point2d, self.fence))

    # The obstacle test is not working. We don't develop further until it is deemed necessary.
    def inside_obstacle(self,point2d,altitude):

        # We cannot be inside an obstacle if there are none
        if self.obstacles is None:
            print 'No obstacles'
            return False

        print 'Obstacles', len(self.obstacles)
        # Check all obstacles
        for obstacle in self.obstacles:
            print 'Obstacle test'
            print obstacle.points
            print obstacle.height
            print point2d
            if self.cn_PnPoly(point2d,obstacle.points):
                print 'Inside 2d obstacle'
                # We might be inside an obstacle
                if altitude < obstacle.height:
                    print 'Inside 3d obstacle'
                    return True

        return False



    # cn_PnPoly(): crossing number test for a point in a polygon
    #     Input:  P = a point,
    #             V[] = vertex points of a polygon
    #     Return: 0 = outside, 1 = inside
    # This code is patterned after [Franklin, 2000]
    # Original copyright notice:
    # Copyright 2001, softSurfer (www.softsurfer.com)
    # This code may be freely used and modified for any purpose
    # providing that this copyright notice is included with it.
    # SoftSurfer makes no warranty for this code, and cannot be held
    # liable for any real or imagined damage resulting from its use.
    # Users of this code must verify correctness for their application.
    # translated to Python by Maciej Kalisiak <mac@dgp.toronto.edu>
    # SOURCE: http://www.dgp.toronto.edu/~mac/e-stuff/point_in_polygon.py
    def cn_PnPoly(self, P, V):
        cn = 0  # the crossing number counter

        # repeat the first vertex at end
        V = tuple(V[:]) + (V[0],)

        # loop through all edges of the polygon
        for i in range(len(V) - 1):  # edge from V[i] to V[i+1]
            if ((V[i][1] <= P[1] and V[i + 1][1] > P[1])  # an upward crossing
                or (V[i][1] > P[1] and V[i + 1][1] <= P[1])):  # a downward crossing
                # compute the actual edge-ray intersect x-coordinate
                vt = (P[1] - V[i][1]) / float(V[i + 1][1] - V[i][1])
                if P[0] < V[i][0] + vt * (V[i + 1][0] - V[i][0]):  # P[0] < intersect
                    cn += 1  # a valid crossing of y=P[1] right of P[0]

        return cn % 2  # 0 if even (out), and 1 if odd (in)


class Obstacle:
    def __init__(self,points, height):
        self.points = points
        self.height = height


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    fence_points = [(1.0,1.0), (10.0,1.0), (10.0,10.0), (1.0,10.0)]
    #obstacle_points = [(4.0,4.0),(6.0,4.0),(4.0,6.0),(6.0,6.0)]
    #obstacles = [Obstacle(obstacle_points,10.0),Obstacle(obstacle_points,10.0)]
    #fence = Geofence(fence_points)
    fence = Geofence("/home/mathias/Dropbox/ROBTEK/9.-semester/RMUASD/UASD_share/geofence/testfence.txt", convert_to_utm=False)
    test_point = (5.0,5.0,100.0)
    print fence.is_point_legal(test_point)

   # print obstacle_points
   # obstacle_fence = Geofence(obstacle_points)
   # print obstacle_fence.is_point_legal(test_point)


if __name__ == "__main__":
    main()
