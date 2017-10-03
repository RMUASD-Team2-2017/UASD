import requests
from requests.auth import HTTPBasicAuth
import json
from gcs.msg import waypoint, path

class web_interface:
    def __init__(self):
        self.username = ""
        self.password = ""
        self.request_URL = ""
        self.request_id = ""
        self.active = False
        self.update_count = 0

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
        r = requests.post(url = get_requests_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        jsonformat = json.loads(r.text) # convert to json

        if(not self.active and int(jsonformat[0]["completed"]) == 0):   # If a mission is not active and the newest request is uncompleted
            print "Got a new request"
            self.request_id = jsonformat[0]["request_id"]
            print self.request_id
            self.active = True

        ### Get the position of an active request ###
        if(self.active):
            payload = {'request_id': self.request_id}
            #headers = {'content-type': 'application/json'}#, 'Accept': 'text/plain'}
            #headers = {'content-type': 'text/html'}
            #headers =  {'content-type': 'multipart/form-data; boundary=----WebKitFormBoundaryVrO2Ct3n1m192cpt'}
            #r_specific = requests.post(url = test_url,auth=HTTPBasicAuth(self.username,self.password),headers=headers,json=payload)
            r_specific = requests.post(url = get_specific_requests_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)

            jsonformat_specific = json.loads(r_specific.text)
            print '*****************************************************'
            #print jsonformat_specific[0]
            # lat = float(jsonformat_specific[0]["loc_lat"])
            # lng = float(jsonformat_specific[0]["loc_lng"])
            # alt = float(jsonformat_specific[0]["altitude"])
            print jsonformat_specific
            lat = float(jsonformat_specific["loc_lat"])
            lng = float(jsonformat_specific["loc_lng"])
            alt = float(jsonformat_specific["altitude"])

            print lat, lng, alt
            self.update_count = self.update_count + 1

        return (self.update_count, lat, lng, alt)

    def setMissionDone(self,data):
        payload = { 'request_id': self.request_id , 'completed': '1'}
        set_mission_done_url = 'https://www.techgen.dk/AED/admin/set_request_completed.php'
        r = requests.post(url = set_mission_done_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
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
        for waypoints in data.path:
            path_string = '(' + str(data.path[i].lat) + ',' + str(data.path[i].lon) + ')'
            i=i+1
        weather_string = 'Great, great weather!'
        payload = { 'drone_id': drone_id, 'cur_lat': data.cur_position.lat, 'cur_lng': data.cur_position.lon, 'target_lat': data.target_position.lat, 'target_lng': data.target_position.lon, 'path': path_string, 'weather': data.weather.data}
        r = requests.post(url = set_preflight_data_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        print r.text

    def setUavState(self,drone_id,state):
        set_uav_state_url = 'https://www.techgen.dk/AED/admin/set_drone_state.php'
        payload = {'drone_id': drone_id, 'state': state}
        r = requests.post(url = set_uav_state_url,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        print r.text

    def setUavCurrentLocation(self,drone_id,data):
        set_uav_current_location = 'https://www.techgen.dk/AED/admin/set_drone_current_position.php'
        payload = {'drone_id': drone_id, 'lat': data.lat, 'lng': data.lon}
        r = requests.post(url = set_uav_current_location,auth=HTTPBasicAuth(self.username,self.password),data=payload)
        print r.text
