#!/usr/bin/python

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

class Target(object):
    id = -1
    user = -1

    def __init__(self, type_s, latitude, longitude, orientation, shape, background_color, alphanumeric,
                 alphanumeric_color, description, autonomous):
        self.type = type_s
        self.latitude = latitude
        self.longitude = longitude
        self.orientation = orientation
        self.shape = shape
        self.shapeColor = background_color
        self.alphanumeric = alphanumeric
        self.alphanumericColor = alphanumeric_color
        self.description = description
        self.autonomous = autonomous

        # Orientation Types: N, NE, E, SE, S, SW, W, NW
        # Shape Types: circle, semicircle, quarter_circle, triangle, square, rectangle, trapezoid, pentagon, hexagon,
        #  heptagon, octagon, star, cross
        # Color Types: white, black, gray, red, blue, green, yellow, purple, brown, orange

    def dict_out(self):
        ret = {}
        ret['type'] = self.type
        ret['autonomous'] = self.autonomous

        #everything else is optional depending on what type we're sending (emergent or standard)
        fields = ['latitude', 'longitude', 'orientation', 'shape', 'shapeColor', 'alphanumeric', 'alphanumericColor', 'description']
        for field in fields:
            val = getattr(self, field)
            if val is not None and val and val != "":
                ret[field] = val

        return ret
