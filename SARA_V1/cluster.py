terfrom constants import *
from math import sin, cos, sqrt, radians
from dronekit import LocationGlobal

"""
class "point":
    initializes variables for point
    __init__:
        measured var: timestamp, gps_reading (origin of lidar measurement), raw (deg, mm, quality), robot_bearing, 
        calculated var: abs_ft, abs_gps
        dbscan var: visited, classification, cluster_name

    getDistanceFrom [X]
        finds distance in feet between two points

    calculateAbsPos: [_]
        given that all angles are in reference from the east, it calculates abs_ft and abs_gps for the point

    printPt [X]
        prints point

class "cluster":
    __init__ [X]
        var: ID (cluster index), list of points, and centroid
    addPoint [X]
        creates a copy of the point and adds it into the cluster
    getCentroid [X]
        averages x and y and returns in array
"""
class point:
    def __init__(self, ts = 0, gps_at_measurement = [], raw_measurement = [], robotBearing = 0, abs_position = []):
        #raw_measurement = [bearing(degree), distance(mm), quality(0-60)]
        self.timestamp = ts
        self.gps_reading = gps_at_measurement
        self.raw = raw_measurement
        self.robot_bearing = robotBearing
        self.abs_ft = abs_position #on 200ft by 200ft map
        self.abs_gps = []

        self.visited = False
        self.classification = ""
        self.cluster_name = ""

    def getDistanceFrom(self, neighbor):
        return sqrt((self.abs_ft[0] - neighbor.abs_ft[0])**2 + (self.abs_ft[1] - neighbor.abs_ft[1])**2)

    def calculateAbsPos(self, gps_origin, field_bearing):
        dist_meters = self.raw[1] /1000.0
        abs_bearing = radians((180.0 + field_bearing + self.robot_bearing + self.raw[0])%360.0)
        dN = cos(abs_bearing) * dist_meters
        dE = sin(abs_bearing) * dist_meters

        self.abs_gps = get_location_metres(self.gps_reading, dN, dE)

        dist_meters = haversine(gps_origin,self.abs_gps)
        rel_bearing = radians((450 - field_bearing + get_bearing(gps_origin,self.abs_gps))%360)
        dN_map = sin(rel_bearing) * dist_meters
        dE_map = cos(rel_bearing) * dist_meters

        self.abs_ft = [dE_map*m_to_ft, dN_map*m_to_ft] 
    
    def printPt(self, calculated = True):
        print "gps_reading:{} raw:{} robot_bearing:{}".format(self.gps_reading, self.raw, self.robot_bearing)
        if(calculated):
            print "abs_ft:{} abs_gps:{}".format(self.abs_ft, self.abs_gps)

class cluster:
    def __init__(self, identity = 0):
        self.ID = identity
        self.points = []
        self.centroid_ft = []

    def addPoint(self, pt):
        self.points.append(point(pt.timestamp, pt.origin, pt.raw, pt.robot_bearing, pt.abs_ft))

    def getCentroid(self):
        #take an average of each point x and y
        centroid = [0,0]
        for pt in self.points: 
            centroid[0] += pt.abs_ft[0]
            centroid[1] += pt.abs_ft[1]

        centroid[0] /= float(len(self.points))
        centroid[1] /= float(len(self.points))

        self.centroid_ft = centroid
        return centroid