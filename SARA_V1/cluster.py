from constants import *
from math import sin, cos, sqrt
from dronekit import LocationGlobal

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
        abs_bearing = ((180 + field_bearing + self.robot_bearing + self.raw[0])%360)*deg_to_rad
        dN = sin(abs_bearing) * dist_meters
        dE = cos(abs_bearing) * dist_meters

        self.abs_gps = get_location_metres(self.gps_reading, dN, dE)

        dist_meters = haversine(gps_origin,self.abs_gps)
        rel_bearing = ((450 - field_bearing + get_bearing(gps_origin,self.abs_gps))%360)*deg_to_rad
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

        centroid[0] /= len(self.points)
        centroid[1] /= len(self.points)

        self.centroid_ft = centroid
        return centroid