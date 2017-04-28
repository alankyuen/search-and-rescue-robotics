import math
from math import radians, cos, sin, asin, sqrt


def haversine(gps1, gps2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    gps1[1], gps1[0], gps2[1], gps2[0] = map(radians, [gps1[1], gps1[0], gps2[1], gps2[0]])

    # haversine formula 
    dlon = gps2[1] - gps1[1]
    dlat = gps2[0] - gps1[0] 
    a = sin(dlat/2)**2 + cos(gps1[0]) * cos(gps2[0]) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371000 # Radius of earth in kilometers. Use 3956 for miles
    return c * r



def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location[0]/180))

    #New position in decimal degrees
    newlat = original_location[0] + (dLat * 180/math.pi)
    newlon = original_location[1] + (dLon * 180/math.pi)
    return [newlat,newlon];
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2[0] - aLocation1[0]
    off_y = aLocation2[1] - aLocation1[1]
    bearing = 90 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;  

def get_distance_metres(aLocation1, aLocation2):
    #WORKS!
    dlat = aLocation2[1]- aLocation1[1]
    dlong = aLocation2[0] - aLocation1[0]
    return sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

c1 = [33.703292, -117.779677]
c2 = [33.704180, -117.779495]

circle_mid = [33.673117, -117.779296]
circle_radius = [33.673066, -117.779222]

print(haversine(c1,c2))
