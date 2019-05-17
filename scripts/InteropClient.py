#!/usr/bin/python
import os
import sys
import getopt
import rospy
import requests
import threading
import urllib
import json
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from rosplane_msgs.msg import State
from uav_msgs.srv import SubmitImage, SubmitImageResponse
from uav_msgs.srv import GetMissionWithId
from uav_msgs.msg import *
from ClientObjects import PostFailedException, Telemetry, Target

class InteropClient(object):
    def __init__(self, server_ip, server_port, server_url, retry_max, 
                    init_lat, init_lon, r_earth, sleep_sec):
        self.SERVERADDR = server_ip
        self.SERVERPORT = server_port
        self.SERVERURL = server_url
        self.GLOBALCOOKIE = None
        self.CONNECTED = False
        self.RETRY_MAX = retry_max
        self.SESSION = requests.Session()
        self.connectedLock = threading.Lock()
        # For NED to LatLon conversions
        self.HOME = [init_lat, init_lon]
        self.R_EARTH = r_earth

        # Connect to server
        rospy.sleep(sleep_sec)
        self.connect()

        # Setup ROS subscribers and publishers
        self.listenerThread = threading.Thread(target=self.listener)
        self.listenerThread.setDaemon(True)
        self.listenerThread.start()
        self.talker()

    def target_submission_handler(self, data):
        # Setup target model and pass to post_target() and post_target_image()

        target = Target(data.type, data.latitude, data.longitude, data.orientation, data.shape, data.background_color, data.alphanumeric, data.alphanumeric_color, data.description, data.autonomous)

        cv2_img = None
        try:
            # Convert the ROS Image message to OpenCV2
            cv2_img = CvBridge().imgmsg_to_cv2(data.image, "bgr8")
        except CvBridgeError, e:
            print("ERROR: converting image message to open cv")
            return

        target_id = None
        try:
            target_id = self.post_target(target) # Throws if posting the target fails after retries
        except PostFailedException:
            # if the post failed, then dont put the image. 
            # imaging subsystem has already saved a copy of the image in odlc format on disk
            # let them know that we failed to submit
            return SubmitImageResponse(False)

        try:
            # If posting the target data succeeded, now post the target image
            self.post_target_image(target_id, cv2_img)
        except PostFailedException:
            # the image failed to post. remove the target from the judge server
            # if possible and return failure
            print("WARN: Failed to post image for target {}".format(target_id))
            if not delete_target(target_id):
                print("WARN: Failed to remove target with id {} from judge server".format(target_id))
            return SubmitImageResponse(False)

        return SubmitImageResponse(True)

    def state_callback(self, data):
        # These come in as NED, so convert to lat / lon
        latlon = self.nedToLatLon((data.position[0], data.position[1]))

        lat = latlon[0]
        lon = latlon[1]
        alt = self.metersToFeet(-data.position[2]) # data.position[2] is negative above ground
        hdg = math.degrees(data.chi % (2 * math.pi))

        telemetry = Telemetry(lat, lon, alt, hdg)

        sendTelemThread = threading.Thread(target=self.post_telemetry, args=(telemetry,))
        sendTelemThread.setDaemon(True)
        sendTelemThread.start()

    def nedToLatLon(self, ned):
        lat = self.HOME[0] + math.degrees(math.asin(ned[0]/self.R_EARTH))
        lon = self.HOME[1] + math.degrees(math.asin(ned[1]/(math.cos(math.radians(self.HOME[0]))*self.R_EARTH)))
        return (lat, lon)

    def listener(self):
        print('Listening')
        rospy.Subscriber("/state", State, self.state_callback) # state info from ros_plane
        rospy.Service("/imaging/target", SubmitImage, self.target_submission_handler) # images + metadata from imaging gui
        #  processing
        rospy.spin()

    def parse_point(self, json):
        point = Point()
        point.latitude = json['latitude']
        point.longitude = json['longitude']

        # Point may optionally have an altitude
        if 'altitude_msl' in json.keys():
            point.altitude = self.feetToMeters(json['altitude_msl'])

        return point

    def parse_ordered_point(self, json):
        ordered_point = OrderedPoint()
        ordered_point.point = self.parse_point(json)
        ordered_point.ordinal = json['order']
        return ordered_point

    def get_mission_with_id_handler(self, req):
        mission_str = self.get_missions()
        obstacles_str = self.get_obstacles()
        json_missions = json.loads(mission_str)

        # Handle multiple missions for testing purposes
        # Just take the first mission that is active
        json_mission = None
        for mission in json_missions:
            if mission["active"]:
                json_mission = mission

        print(json_mission)
        json_obstacles = json.loads(obstacles_str)

        mission_type = req.mission_type
        mission = JudgeMission()

        # Set mission type on response
        mission.mission_type = mission_type

        # Set boundaries (competition boundaries) on response
        for json_point in json_mission['fly_zones'][0]['boundary_pts']:
            ordered = self.parse_ordered_point(json_point)
            mission.boundaries.append(ordered)

        # Set stationary obstacles on response
        for json_obstacle in json_obstacles['stationary_obstacles']:
            point = self.parse_point(json_obstacle)

            obstacle = StationaryObstacle()
            obstacle.point = point
            obstacle.cylinder_height = self.feetToMeters(json_obstacle['cylinder_height'])
            obstacle.cylinder_radius = self.feetToMeters(json_obstacle['cylinder_radius'])

            mission.stationary_obstacles.append(obstacle)

        # Set waypoints based on mission type
        if(mission_type == JudgeMission.MISSION_TYPE_WAYPOINT):

            # Use "mission_waypoints" from judge-provided mission
            for mission_waypoint in json_mission['mission_waypoints']:
                waypoint = self.parse_ordered_point(mission_waypoint)
                mission.waypoints.append(waypoint)

        elif(mission_type == JudgeMission.MISSION_TYPE_DROP):

            # Use "air_drop_pos" from judge-provided mission
            point = self.parse_point(json_mission['air_drop_pos'])
            point.altitude = self.feetToMeters(json_mission['fly_zones'][0]['altitude_msl_min']) # So we know later a baseline of how low we can fly
            ordered = OrderedPoint()
            ordered.point = point
            ordered.ordinal = 1
            mission.waypoints.append(ordered)

        elif(mission_type == JudgeMission.MISSION_TYPE_SEARCH):
            # Use search grid boundaries as waypoints. Caller will generate actual search path within this.
            for search_grid_point in json_mission['search_grid_points']:
                point = self.parse_ordered_point(search_grid_point)
                mission.waypoints.append(point)

        elif(mission_type == JudgeMission.MISSION_TYPE_OFFAXIS):
            # Get off axis position
            point = self.parse_point(json_mission['off_axis_odlc_pos'])
            ordered = OrderedPoint()
            ordered.point = point
            ordered.ordinal = 1
            mission.waypoints.append(ordered)

        elif(mission_type == JudgeMission.MISSION_TYPE_EMERGENT):
            # Get emergent position
            point = self.parse_point(json_mission["emergent_last_known_pos"])
            ordered = OrderedPoint()
            ordered.point = point
            ordered.ordinal = 1
            mission.waypoints.append(ordered)

        return mission

    def talker(self):
        print('Talking')

        # Init the GetMission service handler
        s = rospy.Service("get_mission_with_id", GetMissionWithId, self.get_mission_with_id_handler)

        rate = rospy.Rate(1) # not sure if we need this. i think this is from when we tried to publish moving obstacle info every second

        string = self.get_obstacles()
        json_obstacles = json.loads(string)
        print("Got obstacles!")

    def is_connected(self):
        # global CONNECTED

        self.connectedLock.acquire()
        connected = self.CONNECTED
        self.connectedLock.release()

        return connected

    def set_is_connected(self, connected):
        # global CONNECTED

        self.connectedLock.acquire()
        self.CONNECTED = connected
        self.connectedLock.release()

    def connect(self):
        # params = {"username": "testuser", "password": "testpass"}
        params = {"username": "testuser", "password": "testpass"}
        retry_count = 0
        while not self.is_connected() and retry_count < self.RETRY_MAX:
            retry_count+=1

            try:
                # print('Logging in')
                headers = {"Content-Type": "application/json"}
                response = self.SESSION.post(self.SERVERURL + '/api/login', headers=headers, json=params)

                if response.status_code == 200:
                    self.GLOBALCOOKIE = response.headers.get('Set-Cookie')

                    print('Successfully Logged In')
                    self.set_is_connected(True)
                else:
                    print("response code::{}".format(response.status_code))
                    raise Exception('Error Logging In')
            except Exception as e:
                # print('Connection Error: ' + str(e))
                self.set_is_connected(False)

        if retry_count >= self.RETRY_MAX:
            print("could not connect to server. (address=" + self.SERVERADDR + ", port=" + str(self.SERVERPORT) + ", " + ", ".join(str(params).split("&")) + "). exiting...")
            exit()

    def send_request(self, method, resource, params, headers):
        response = None

        while True:
            if not self.is_connected():
                print('Connecting')
                self.connect()

            url = self.SERVERURL+resource

            if method == 'GET':
                response = self.SESSION.get(self.SERVERURL+resource, headers=headers)
            elif method == 'POST':
                response = self.SESSION.post(self.SERVERURL+resource, headers=headers, data=params)
                print("Successfully posted: {}, {}".format(resource,headers))
            elif method == 'PUT':
                response = self.SESSION.put(self.SERVERURL+resource, headers=headers, data=params)
            elif method == 'DELETE':
                response = self.SESSION.delete(self.SERVERURL+resource, headers=headers, data=params)

            if response.status_code == 200 or response.status_code == 201:
                break
            elif response.status_code == 400:
                print('400 - Bad Request')
                print('url:' + url)
                print(params)
                print(headers)
                break
            elif response.status_code == 403:
                self.set_is_connected(False)  # Retry but create a new connection and login again first
                print('403 - Forbidden: Was the cookie sent?')
                print('url:' + url)
                print(headers)
            elif response.status_code == 404:
                print('404 - Not Found: {url:' + url + '}')
                break
            elif response.status_code == 405:
                print('405 - Invalid Request: {url:' + url + ', method:' + method + '}')
                break
            elif response.status_code == 413:
                print('413 - Image is too large, it needs to be < 1MB')
                break
            elif response.status_code == 500:
                print('500 - SERVER ERROR')
                break

        return response

    def get_obstacles(self):
        response = self.send_request('GET', '/api/obstacles', None, headers={'Cookie': self.GLOBALCOOKIE})
        return response.text

    def get_missions(self):
        response = self.send_request('GET', '/api/missions', None, headers={'Cookie': self.GLOBALCOOKIE})
        return response.text

    def post_telemetry(self, telemetry):
        params = urllib.urlencode(
                    {'latitude': telemetry.latitude, 'longitude': telemetry.longitude, 'altitude_msl': telemetry.altitude,
                        'uas_heading': telemetry.heading})
        headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain", 'Cookie': self.GLOBALCOOKIE}
        response = self.send_request('POST', '/api/telemetry', params, headers)

    def post_target(self, target):
        params = {'type': target.type, 'latitude': target.latitude, 'longitude': target.longitude,
                  'orientation': target.orientation, 'shape': target.shape, 'background_color': target.background_color,
                  'alphanumeric': target.alphanumeric, 'alphanumeric_color': target.alphanumeric_color,
                  'description': target.description, 'autonomous':target.autonomous}

        json_params = json.dumps(params)

        headers = {"Content-Type": "application/json", "Accept": "text/plain", 'Cookie': self.GLOBALCOOKIE}
        response = self.send_request('POST', '/api/odlcs', json_params, headers)

        for retry in range(self.RETRY_MAX):
            if response.status_code == 201:
                print("Target was submitted successfully on try {}!".format(retry + 1))
                return response.json()['id']
            else:
                print("Something went wrong with posting a target, trying again")
                # actually try again...
                response = self.send_request('POST', '/api/odlcs', json_params, headers)

        print("Target failed after {} tries".format(self.RETRY_MAX))
        raise PostFailedException()

    def delete_target(self, target_id):
        headers = {'Cookie': self.GLOBALCOOKIE}
        response = self.send_request('DELETE', '/api/odlcs/' + str(target_id), None, headers)
        return response.status_code == 200

    def post_target_image(self, target_id, image):
        encoded_image = cv2.imencode(".jpg", image)[1].tostring()

        headers = {"Content-Type": "image/jpeg", 'Cookie': self.GLOBALCOOKIE}
        response = self.send_request('POST', '/api/odlcs/' + str(target_id) + '/image', encoded_image, headers)

        for retry in range(self.RETRY_MAX):
            if response.status_code == 200:
                print("Target image was submitted successfully on try {}!".format(retry + 1))
                return
            else:
                print("Something went wrong with posting an image, trying again")

        print("Failed to post target image with id: {}".format(target_id))
        raise PostFailedException()

    def feetToMeters(self, feet):
        return feet * 0.3048

    def metersToFeet(self, meters):
        return meters * 3.28084

if __name__ == '__main__':
    # initialize node
    rospy.init_node('interop_client', anonymous=True)

    # load parameters
    server_ip = rospy.get_param("~SERVER_IP")
    server_port = rospy.get_param("~SERVER_PORT")
    server_url = "http://" + server_ip + ":" + str(server_port)
    retry_max = rospy.get_param("~MAX_RETRIES")
    init_lat = rospy.get_param("~init_lat")
    init_lon = rospy.get_param("~init_lon")
    r_earth = rospy.get_param("~r_earth")
    sleep_sec = rospy.get_param("~sleep_sec")

    # initialize client
    client = InteropClient(server_ip, server_port, server_url, retry_max, 
                init_lat, init_lon, r_earth, sleep_sec)

    # keep the program running until manually killed
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Kill signal received.')

    print('Shutting down interop client...')
