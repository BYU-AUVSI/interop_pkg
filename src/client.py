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
from rosflight_msgs.msg import State
from sniper_cam.msg import interopImages

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
SERVERADDR = assert_param('127.0.0.1', 'INTEROP_SERVER', 'SERVER')
SERVERPORT = int(assert_param(8000, 'SERVER_PORT', 'PORT'))
SERVERURL = "http://" + SERVERADDR + ":" + str(SERVERPORT)
GLOBALCOOKIE = None
CONNECTED = False
RETRY_MAX = 3
SESSION = requests.Session()

new_lat = False
new_long = False
new_alt = False
new_hdg = False

cookieLock = threading.Lock()
connectedLock = threading.Lock()


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
    id = post_target(target)
    imgname = "target_" + str(id) + ".jpeg"
    try:
        # Convert the ROS Image message to OpenCV2
        cv2_img = CvBridge().imgmsg_to_cv2(data.image, "bgr8")
    except CvBridgeError, e:
        print("ERROR: saving target "+ str(id) + " image")
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite(imgname, cv2_img)
    post_target_image(id, imgname)
    # delete the image now that we're done using it
    os.remove(imgname)

def state_callback(data):
    telem = dict()
    telem['lat'] = data.position[0]
    telem['long'] = data.position[1]
    telem['alt'] = data.position[2]
    telem['hdg'] = math.degrees(data.chi % (2 * math.pi))
    update_telemetry(telem)
    # rospy.logdebug(rospy.get_caller_id() + "GPS Latitude: %s, Longitude: %s, Altitude: %s, Heading %s", telem["lat"], telem["long"], telem["alt"], telem['hdg'])


def listener():
    print('Listening')
    rospy.Subscriber("/gps_state", State, state_callback) # state info from ros_plane
    rospy.Subscriber("/plans", interopImages, target_callback) # images + metadata from imaging gui
    #  processing
    rospy.spin()


def update_telemetry(data):
    global telemetry
    global new_lat
    global new_long
    global new_alt
    global new_hdg

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

    if new_lat and new_long and new_alt and new_hdg:
        sendTelemThread = threading.Thread(target=send_telemetry)
        sendTelemThread.setDaemon(True)
        sendTelemThread.start()


def talker():
    print('Talking')
    obstacles = rospy.Publisher('obstacles', String, queue_size=10)
    moving_obstacles = rospy.Publisher('obstacles/moving', String, queue_size=10)
    stationary_obstacles = rospy.Publisher('obstacles/stationary', String, queue_size=10)
    missions = rospy.Publisher('missions', String, queue_size=10)
    rate = rospy.Rate(5)

    print "fetching, parsing, and transmitting obstacle and mission data..."
    while not rospy.is_shutdown():
        string = get_obstacles()
        json_obstacles = json.loads(string)
        rospy.logdebug(string)
        obstacles.publish(str(string))
        moving_obstacles.publish(json.dumps(json_obstacles['moving_obstacles']))
        stationary_obstacles.publish(json.dumps(json_obstacles['stationary_obstacles']))

        string = get_missions()
        rospy.logdebug(string)
        missions.publish(str(string))

        rate.sleep()


def get_cookie():
    global GLOBALCOOKIE

    cookieLock.acquire()
    cookie = GLOBALCOOKIE
    cookieLock.release()

    return cookie


def set_cookie(cookie):
    global GLOBALCOOKIE

    cookieLock.acquire()
    GLOBALCOOKIE = cookie
    cookieLock.release()


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
    post_telemetry()
    new_lat = False
    new_long = False
    new_alt = False
    new_hdg = False


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
    response = send_request('POST', '/api/targets', json_params, headers)

    if response.status_code == 201:
        print("Target was submitted successfully!")
        return response.json()['id']
    else:
        print("Something went wrong with posting a target!")
        return -1


def post_target_image(target_id, image_name):
    with open(image_name, "rb") as image_file:
        encoded_image = image_file.read()

    headers = {"Content-Type": "image/jpeg", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/targets/' + str(target_id) + '/image', encoded_image, headers)

    if response.status_code == 200:
        print("Target image was submitted successfully!")
    else:
        print("Something went wrong with posting an image!")
        print(response.text)


if __name__ == '__main__':
    rospy.init_node('interop_client', anonymous=True)
    connect()

    listenerThread = threading.Thread(target=listener)
    listenerThread.setDaemon(True)
    listenerThread.start()
    talker()
