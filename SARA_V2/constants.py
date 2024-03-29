from math import radians, degrees, cos, sin, asin, sqrt, pi, atan2
from dronekit import LocationGlobal

deg_to_rad = 0.0174533
m_to_ft = 3.28084
ft_to_m = 0.3048

def printGPS(gps):
    print"GPS:[{},{}]".format(gps.lat,gps.lon)
def getDist(pt1,pt2):
	return sqrt((pt1[1]-pt2[1])**2 + (pt1[0]-pt2[0])**2)
	
def haversine(gps1,gps2):
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [gps1.lon, gps1.lat, gps2.lon, gps2.lat])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378137.0 # Radius of earth in kilometers. Use 3956 for miles
    return c * r

def calculateGPSPos(dist_meters,degree,robot_bearing, gps_origin, field_bearing):
    abs_bearing = radians(field_bearing + robot_bearing + degree)%360.0
    dN = cos(abs_bearing) * dist_meters
    dE = sin(abs_bearing) * dist_meters
    abs_gps = get_location_metres(self.gps_reading, dN, dE)
    return abs_gps

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius * cos(pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/pi)
    newlon = original_location.lon + (dLon * 180/pi)
    return LocationGlobal(newlat,newlon, original_location.alt)

def get_bearing(pointA, pointB):

    lat1 = radians(pointA.lat)
    lat2 = radians(pointB.lat)

    diffLong = radians(pointB.lon - pointA.lon)

    x = sin(diffLong) * cos(lat2)
    y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(diffLong))

    initial_bearing = atan2(x, y)

    initial_bearing = degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def calcGPS_from_map(origin, field_bearing, map_pos):
    map_point_bearing = degrees(atan2(map_pos[1],-map_pos[0]))
    map_point_dist = sqrt(map_pos[0]**2 + map_pos[1]**2) * ft_to_m

    abs_point_bearing = radians(field_bearing + 270 + map_point_bearing)

    dN = cos(abs_point_bearing) * map_point_dist
    dE = sin(abs_point_bearing) * map_point_dist

    return get_location_metres(origin,dN,dE)