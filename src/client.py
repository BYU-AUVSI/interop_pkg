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
from sniper_cam.msg import interopImages
from uav_msgs.srv import GetMissionWithId
from uav_msgs.msg import *

# set these values according to current environment variables
# if environment variables don't exist, use default values
def assert_param(default, *args):
    for arg in args:
        if (os.environ.get(arg) != None):
            return os.environ.get(arg)
    if (default != None):
        print("could not find value for " + args[0] + ". Using default value of " + str(default))
        return default
    else:
        print("could not find default value for " + args[0])
        exit()
SERVERADDR = assert_param('localhost', 'INTEROP_SERVER', 'SERVER')
SERVERPORT = int(assert_param(8000, 'SERVER_PORT', 'PORT'))
SERVERURL = "http://" + SERVERADDR + ":" + str(SERVERPORT)
GLOBALCOOKIE = None
CONNECTED = False
RETRY_MAX = 3
SESSION = requests.Session()

BACKUP_OBJECT_PATH = os.path.expanduser("~/Desktop/objects/") # Where to write submitted objects when judges go down
unique_id = 0 # id that we will use to write images when we don't get one assigned from server

new_lat = False
new_long = False
new_alt = False
new_hdg = False

connectedLock = threading.Lock()
telemetryLock = threading.Lock()

# For NED to LatLon
HOME = [38.144692, -76.428007]
R_EARTH = 6370027

# A call to post a target to the judges failed after RETRY_MAX retries
class PostFailedException(Exception):
    pass


class Telemetry(object):
    def __init__(self, latitude, longitude, altitude, heading):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

    def printTelemetry(self):
        print('Latitude: ', self.latitude)
        print('Longitude: ', self.longitude)
        print('Altitude: ', self.altitude)
        print('Heading: ', self.heading)

    # use __getattr__('heading') or other attributes for the data
    # use __set__('heading', 90) or other attributes to set data

# global telemetry object
telemetry = Telemetry(0, 0, 0, 0)


class Target(object):
    id = -1
    user = -1

    def __init__(self, type, latitude, longitude, orientation, shape, background_color, alphanumeric,
                 alphanumeric_color, description, autonomous):
        self.type = type
        self.latitude = latitude
        self.longitude = longitude
        self.orientation = orientation
        self.shape = shape
        self.background_color = background_color
        self.alphanumeric = alphanumeric
        self.alphanumeric_color = alphanumeric_color
        self.description = description
        self.autonomous = autonomous

        # Orientation Types: N, NE, E, SE, S, SW, W, NW
        # Shape Types: circle, semicircle, quarter_circle, triangle, square, rectangle, trapezoid, pentagon, hexagon,
        #  heptagon, octagon, star, cross
        # Color Types: white, black, gray, red, blue, green, yellow, purple, brown, orange

def target_callback(data):
    # Setup target model and pass to post_target() and post_target_image()
    # TODO: create thread for this
    orientation_deg = data.orientation % 360
    if (orientation_deg <= 22.5 or orientation_deg >= 337.5):
        orientation = "n"
    elif (orientation_deg >= 22.5 and orientation_deg <= 67.5):
        orientation = "nw"
    elif (orientation_deg >= 67.5 and orientation_deg <= 112.5):
        orientation = "w"
    elif (orientation_deg >= 112.5 and orientation_deg <= 157.5):
        orientation = "sw"
    elif (orientation_deg >= 157.5 and orientation_deg <= 202.5):
        orientation = "s"
    elif (orientation_deg >= 202.5 and orientation_deg <= 247.5):
        orientation = "se"
    elif (orientation_deg >= 247.5 and orientation_deg <= 292.5):
        orientation = "e"
    else:
        orientation = "ne"

    target = Target(data.type, data.gps_lati, data.gps_longit, orientation, data.target_shape, data.target_color, data.symbol, data.symbol_color, data.description, data.autonomous)

    cv2_img = None
    try:
        # Convert the ROS Image message to OpenCV2
        cv2_img = CvBridge().imgmsg_to_cv2(data.image, "bgr8")
    except CvBridgeError, e:
        print("ERROR: converting image message to open cv")
        return

    target_id = None
    try:
        target_id = post_target(target) # Throws if posting the target fails after retries
    except PostFailedException:
        # Write target / target image to object file
        target_id = pick_unique_id()
        print("Writing target data and image to file with generated id: {}".format(target_id))
        write_target_data_to_file(target, target_id)
        write_target_image_to_file(cv2_img, target_id)
        return

    # If posting the target data succeeded, now post the target image
    post_target_image(target_id, cv2_img)

def write_target_data_to_file(target_data, target_id):
    make_directory_if_not_exists(BACKUP_OBJECT_PATH)

    name = BACKUP_OBJECT_PATH + str(target_id) + ".json"

    # Write target_data as json to name path
    params = {'type': target_data.type, 'latitude': target_data.latitude, 'longitude': target_data.longitude,
              'orientation': target_data.orientation, 'shape': target_data.shape, 'background_color': target_data.background_color,
              'alphanumeric': target_data.alphanumeric, 'alphanumeric_color': target_data.alphanumeric_color,
              'description': target_data.description, 'autonomous':target_data.autonomous}

    json_params = json.dumps(params)
    with open(name, "w") as f:
        f.write(json_params)

def write_target_image_to_file(image, target_id):
    make_directory_if_not_exists(BACKUP_OBJECT_PATH)

    name = BACKUP_OBJECT_PATH + str(target_id) + ".jpg"
    cv2.imwrite(name, image)

def make_directory_if_not_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

def pick_unique_id():
    global unique_id
    unique_id += 1
    return unique_id

def state_callback(data):
    telem = dict()

    # These come in as NED, so convert to lat / lon
    latlon = nedToLatLon((data.position[0], data.position[1]))

    telem['lat'] = latlon[0]
    telem['long'] = latlon[1]
    telem['alt'] = metersToFeet(-data.position[2]) # data.position[2] is negative above ground
    telem['hdg'] = math.degrees(data.chi % (2 * math.pi))
    update_telemetry(telem)

def nedToLatLon(ned):
    lat = HOME[0] + math.degrees(math.asin(ned[0]/R_EARTH))
    lon = HOME[1] + math.degrees(math.asin(ned[1]/(math.cos(math.radians(HOME[0]))*R_EARTH)))
    return (lat, lon)

def listener():
    print('Listening')
    rospy.Subscriber("/state", State, state_callback) # state info from ros_plane
    rospy.Subscriber("/plans", interopImages, target_callback) # images + metadata from imaging gui
    #  processing
    rospy.spin()


def update_telemetry(data):
    global telemetry
    global new_lat
    global new_long
    global new_alt
    global new_hdg

    # Take a lock before accessing these vars (shared with other threads)
    telemetryLock.acquire()
    if 'lat' in data:
        telemetry.latitude = data['lat']
        new_lat = True
    if 'long' in data:
        telemetry.longitude = data['long']
        new_long = True
    if 'alt' in data:
        telemetry.altitude = data['alt']
        new_alt = True
    if 'hdg' in data:
        telemetry.heading = data['hdg']
        new_hdg = True

    newData = new_lat and new_long and new_alt and new_hdg
    telemetryLock.release()

    if newData:
        sendTelemThread = threading.Thread(target=send_telemetry)
        sendTelemThread.setDaemon(True)
        sendTelemThread.start()

def parse_point(json):
    point = Point()
    point.latitude = json['latitude']
    point.longitude = json['longitude']

    # Point may optionally have an altitude
    if 'altitude_msl' in json.keys():
        point.altitude = feetToMeters(json['altitude_msl'])

    return point

def parse_ordered_point(json):
    ordered_point = OrderedPoint()
    ordered_point.point = parse_point(json)
    ordered_point.ordinal = json['order']
    return ordered_point

def get_mission_with_id_handler(req):
    mission_str = get_missions()
    obstacles_str = get_obstacles()
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
        ordered = parse_ordered_point(json_point)
        mission.boundaries.append(ordered)

    # Set stationary obstacles on response
    for json_obstacle in json_obstacles['stationary_obstacles']:
        point = parse_point(json_obstacle)

        obstacle = StationaryObstacle()
        obstacle.point = point
        obstacle.cylinder_height = feetToMeters(json_obstacle['cylinder_height'])
        obstacle.cylinder_radius = feetToMeters(json_obstacle['cylinder_radius'])

        mission.stationary_obstacles.append(obstacle)

    # Set waypoints based on mission type
    if(mission_type == JudgeMission.MISSION_TYPE_WAYPOINT):

        # Use "mission_waypoints" from judge-provided mission
        for mission_waypoint in json_mission['mission_waypoints']:
            waypoint = parse_ordered_point(mission_waypoint)
            mission.waypoints.append(waypoint)

    elif(mission_type == JudgeMission.MISSION_TYPE_DROP):

        # Use "air_drop_pos" from judge-provided mission
        point = parse_point(json_mission['air_drop_pos'])
        point.altitude = feetToMeters(json_mission['fly_zones'][0]['altitude_msl_min']) # So we know later a baseline of how low we can fly
        ordered = OrderedPoint()
        ordered.point = point
        ordered.ordinal = 1
        mission.waypoints.append(ordered)

    elif(mission_type == JudgeMission.MISSION_TYPE_SEARCH):
        # Use search grid boundaries as waypoints. Caller will generate actual search path within this.
        for search_grid_point in json_mission['search_grid_points']:
            point = parse_ordered_point(search_grid_point)
            mission.waypoints.append(point)

    elif(mission_type == JudgeMission.MISSION_TYPE_OFFAXIS):
        # Get off axis position
        point = parse_point(json_mission['off_axis_odlc_pos'])
        ordered = OrderedPoint()
        ordered.point = point
        ordered.ordinal = 1
        mission.waypoints.append(ordered)

    elif(mission_type == Judge.MISSION_TYPE_EMERGENT):
        # Get emergent position
        point = parse_point(json_mission["emergent_last_known_pos"])
        ordered = OrderedPoint()
        ordered.point = point
        ordered.ordinal = 1
        mission.waypoints.append(ordered)

    return mission

def talker():
    print('Talking')

    # Init the GetMission service handler
    s = rospy.Service("get_mission_with_id", GetMissionWithId, get_mission_with_id_handler)

    moving_obstacles = rospy.Publisher('moving_obstacles', MovingObstacleCollection, queue_size=1)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        string = get_obstacles()
        json_obstacles = json.loads(string)
        json_moving_obstacles = json_obstacles["moving_obstacles"]

        collection = MovingObstacleCollection()
        for json_obstacle in json_moving_obstacles:
            point = parse_point(json_obstacle)
            obstacle = MovingObstacle()
            obstacle.point = point
            obstacle.sphere_radius = feetToMeters(json_obstacle["sphere_radius"])
            collection.moving_obstacles.append(obstacle)

        moving_obstacles.publish(collection)
        rate.sleep()

def get_cookie():
    global GLOBALCOOKIE
    return GLOBALCOOKIE


def set_cookie(cookie):
    global GLOBALCOOKIE
    GLOBALCOOKIE = cookie


def is_connected():
    global CONNECTED

    connectedLock.acquire()
    connected = CONNECTED
    connectedLock.release()

    return connected


def set_is_connected(connected):
    global CONNECTED

    connectedLock.acquire()
    CONNECTED = connected
    connectedLock.release()


def connect():
    params = urllib.urlencode({'username': 'testuser', 'password': 'testpass'})
    retry_count = 0
    while not is_connected() and retry_count < RETRY_MAX:
        retry_count+=1

        try:
            # print('Logging in')
            headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain"}
            response = SESSION.post(SERVERURL+'/api/login', headers=headers, data=params)

            if response.status_code == 200:
                set_cookie(response.headers.get('Set-Cookie'))
                # print('Cookie:', get_cookie())

                print('Successfully Logged In')
                set_is_connected(True)
            else:
                raise Exception('Error Logging In')
        except Exception as e:
            print('Connection Error: ' + str(e))
            set_is_connected(False)

    if retry_count >= RETRY_MAX:
        print("could not connect to server. (address=" + SERVERADDR + ", port=" + str(SERVERPORT) + ", " + ", ".join(str(params).split("&")) + "). exiting...")
        exit()


def send_request(method, resource, params, headers):
    response = None

    while True:
        if not is_connected():
            print('Connecting')
            connect()

        url = SERVERURL+resource

        if method == 'GET':
            response = SESSION.get(SERVERURL+resource, headers=headers)
        elif method == 'POST':
            #print("Posting {}, {}, {}".format(SERVERURL + resource, headers, params))
            response = SESSION.post(SERVERURL+resource, headers=headers, data=params)
        elif method == 'PUT':
            response = SESSION.put(SERVERURL+resource, headers=headers, data=params)

        if response.status_code == 200 or response.status_code == 201:
            # print('200/201 - Success') # let's not print this for every http request
            break
        elif response.status_code == 400:
            print('400 - Bad Request')
            print('url:' + url)
            print(params)
            print(headers)
            break
        elif response.status_code == 403:
            set_is_connected(False)  # Retry but create a new connection and login again first
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


def get_obstacles():
    response = send_request('GET', '/api/obstacles', None, headers={'Cookie': get_cookie()})
    return response.text


def get_missions():
    response = send_request('GET', '/api/missions', None, headers={'Cookie': get_cookie()})
    return response.text


def send_telemetry():
    global telemetry
    global new_lat
    global new_long
    global new_alt
    global new_hdg

    # telemetry.printTelemetry()
    telemetryLock.acquire()
    new_lat = False
    new_long = False
    new_alt = False
    new_hdg = False
    post_telemetry()
    telemetryLock.release()


def post_telemetry():
    global telemetry
    params = urllib.urlencode(
                {'latitude': telemetry.latitude, 'longitude': telemetry.longitude, 'altitude_msl': telemetry.altitude,
                    'uas_heading': telemetry.heading})
    headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/telemetry', params, headers)


def post_target(target):
    params = {'type': target.type, 'latitude': target.latitude, 'longitude': target.longitude,
              'orientation': target.orientation, 'shape': target.shape, 'background_color': target.background_color,
              'alphanumeric': target.alphanumeric, 'alphanumeric_color': target.alphanumeric_color,
              'description': target.description, 'autonomous':target.autonomous}

    json_params = json.dumps(params)

    headers = {"Content-Type": "application/json", "Accept": "text/plain", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/odlcs', json_params, headers)

    for retry in range(RETRY_MAX):
        if response.status_code == 201:
            print("Target was submitted successfully on try {}!".format(retry + 1))
            return response.json()['id']
        else:
            print("Something went wrong with posting a target, trying again")

    print("Target failed after {} tries".format(RETRY_MAX))
    raise PostFailedException()

def post_target_image(target_id, image):
    encoded_image = cv2.imencode(".jpg", image)[1].tostring()

    headers = {"Content-Type": "image/jpeg", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/odlcs/' + str(target_id) + '/image', encoded_image, headers)

    for retry in range(RETRY_MAX):
        if response.status_code == 200:
            print("Target image was submitted successfully on try {}!".format(retry + 1))
            return
        else:
            print("Something went wrong with posting an image, trying again")
    print("Writing target image to file with id: {}".format(target_id))
    write_target_image_to_file(image, target_id)

def feetToMeters(feet):
    return feet * 0.3048

def metersToFeet(meters):
    return meters * 3.28084

if __name__ == '__main__':
    rospy.init_node('interop_client', anonymous=True)
    connect()

    listenerThread = threading.Thread(target=listener)
    listenerThread.setDaemon(True)
    listenerThread.start()
    talker()
