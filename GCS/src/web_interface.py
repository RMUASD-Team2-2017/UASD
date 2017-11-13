import requests
from requests.auth import HTTPBasicAuth
import json
import time
from rospy_message_converter import message_converter
from gcs.msg import waypoint, path

class web_interface:
    def __init__(self):
        self.username = ""
        self.password = ""
        self.request_URL = ""
        self.request_id = ""
        self.active = False
        self.update_count = -1
        self.position_publish_limit_counter = 0
        self.target_lat = 0.0
        self.target_lng = 0.0

    def setAuthentication(self,_username,_password):
        self.username = _username
        self.password = _password

    def getDeployRequest(self):
        get_requests_url = 'https://www.techgen.dk/AED/admin/get_requests.php'
        get_specific_requests_url = 'https://www.techgen.dk/AED/admin/get_specific_request_single.php'
        test_url = 'http://httpbin.org/post'

        lat = 0.0
        lng = 0.0
        alt = 0.0
        ### Get all requests ###
        payload = {'show_uncompleted': 1}
        r = ''
        try:
            r = requests.post(url = get_requests_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        except:
            print 'Unexpected error in polling get_requests'
            return (-5, lat, lng, alt)
        #print "The request status is", r.status_code
        if r.text == '0' or int(r.status_code) != 200: # Handle empty response
            return (-1, lat, lng, alt)
        jsonformat = ''
        try:
            jsonformat = json.loads(r.text) # convert to json
        except:
            print 'Unexpected error in parsing r.text as json'
            return (-2, lat, lng, alt)
        if(not self.active and int(jsonformat[0]["completed"]) == 0):   # If a mission is not active and the newest request is uncompleted
            print "Got a new request"
            self.request_id = jsonformat[0]["request_id"]
            print self.request_id
            self.active = True

        ### Get the position of an active request ###
        if(self.active):
            payload = {'request_id': self.request_id}
            r_specific = ''
            try:
                r_specific = requests.post(url = get_specific_requests_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
            except:
                print 'Unexpected error in polling get_specific_request'
                return (-6, lat, lng, alt)
            print "The position status is", r.status_code
            if r_specific.text == 'null' or r_specific.text == '0' or int(r.status_code) != 200:
                return (-3, lat, lng, alt)
            jsonformat_specific = ''
            try:
                jsonformat_specific = json.loads(r_specific.text)
            except:
                print 'Unexpected error in parsing r_specific as json'
                return (-4, lat, lng, alt)
            print '*****************************************************'
            # If using the non-single get_specific_request_url, use the indexing below
            #print jsonformat_specific[0]
            # lat = float(jsonformat_specific[0]["loc_lat"])
            # lng = float(jsonformat_specific[0]["loc_lng"])
            # alt = float(jsonformat_specific[0]["altitude"])
            #print jsonformat_specific
            lat = float(jsonformat_specific["loc_lat"])
            lng = float(jsonformat_specific["loc_lng"])
            alt = float(jsonformat_specific["altitude"])
            target_lat = lat
            target_lng = lng
            print lat, lng, alt
            self.update_count = self.update_count + 1

        return (self.update_count, lat, lng, alt)

    def setMissionDone(self,data):
        # UNCOMMENT THIS TO TEST. IT WILL ALLOW TO COMPLETE MORE MISSIONS WITHOUT RESTART OF THIS NODE
        # It may send a deploy request immideately before the mission is marked complete.
        #self.active = False
        #self.update_count = -1
        payload = { 'request_id': self.request_id , 'completed': '1'}
        set_mission_done_url = 'https://www.techgen.dk/AED/admin/set_request_completed.php'
        r = ''
        complete = False
        while not complete:
            try:
                r = requests.post(url = set_mission_done_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
                json_format = json.loads(r.text)
                if json_format["status"] == 1:
                    complete = True
            except:
                print 'Unexpected error in set_mission_done'
            if not complete:
                time.sleep(1)


        print '#################################################################################'
        print self.username, self.password
        print self.request_id
        print r.text
        print 'Mission marked as completed'
        print '#################################################################################'

    def setPreflightData(self,drone_id,data):
        set_preflight_data_url = 'https://www.techgen.dk/AED/admin/set_drone_preflight.php'
        path_string = ''
        i = 0
        #dict = message_converter.convert_ros_message_to_dictionary(data) This is really, really smart! We might need it at some point.
        # It converts ros messages to dict+json  (json: using a different function) and vice versa. Installed with sudo pip install message_converter. Examples: https://github.com/baalexander/rospy_message_converter
        for waypoint in data.path_waypoints.path:
            path_string = path_string + '(' + str(data.path_waypoints.path[i].lat) + ',' + str(data.path_waypoints.path[i].lon) + ')'
            i=i+1
        cur_lat = data.path_waypoints.path[0].lat
        cur_lon = data.path_waypoints.path[0].lon
        target_lat = data.path_waypoints.path[i-1].lat
        target_lon = data.path_waypoints.path[i-1].lon
        weather = {'temp': data.temperature, 'humidity': data.humidity, 'wind': data.wind}
        weather_string = json.dumps(weather)
        payload = { 'drone_id': drone_id, 'cur_lat': cur_lat, 'cur_lng': cur_lon, 'target_lat': target_lat, 'target_lng': target_lon, 'path': path_string, 'weather': weather_string}
        r = ''
        complete = False
        while not complete:
            try:
                r = requests.post(url = set_preflight_data_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
                json_format = json.loads(r.text)
                if json_format["status"] == 1:
                    complete = True
            except:
                print 'Unexpected error in set_mission_done'
            if not complete:
                time.sleep(1)


        print '#################################################################################'
        print 'Preflight information transmitted'
        print '#################################################################################'

    def setUavState(self,drone_id,state): #Check this
        set_uav_state_url = 'https://www.techgen.dk/AED/admin/set_drone_state.php'
        payload = {'drone_id': drone_id, 'state': state}
        r = ''
        try:
            r = requests.post(url = set_uav_state_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        except:
            print 'Unexpected error in polling set_uav_state'
            return -9
        #print 'setUavState', r.text
        if state == 'transport':
            drone_id = 1
            eta = 0
            self.setRequestDroneIdEta(drone_id,eta)

    def setUavCurrentLocation(self,drone_id,data): # Change to correct waypoint type as responded
        if self.position_publish_limit_counter == 0: # Limit to 1 hz
            set_uav_current_location = 'https://www.techgen.dk/AED/admin/set_drone_current_position.php'
            payload = {'drone_id': drone_id, 'lat': data.latitude, 'lng': data.longitude}
            r = ''
            try:
                r = requests.post(url = set_uav_current_location,auth=HTTPBasicAuth(self.username,self.password),data=payload)
            except:
                print 'Unexpected in set_uav_current_location'
                return -10
            print 'setCurrentLocation result: ', r.text
        # Count up but limit. Input: 20 Hz. WIth % 10 -> output: 2 Hz
        self.position_publish_limit_counter = (self.position_publish_limit_counter + 1) % 20

    def setRequestDroneIdEta(self,drone_id,eta):
        set_id_and_eta_url = 'https://www.techgen.dk/AED/admin/set_request_drone_eta.php'
        payload = {'request_id': self.request_id, 'drone_id': drone_id, 'eta': eta}
        r = ''
        try:
            r = requests.post(url = set_id_and_eta_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        except:
            print 'Unexpected error in polling get_requests'
            return -11
        print 'Set drone id and eta'
