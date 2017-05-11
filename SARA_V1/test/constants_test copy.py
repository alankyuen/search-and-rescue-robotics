from math import radians, cos, sin, asin, sqrt, pi, atan2
import utm
deg_to_rad = 0.0174533
m_to_ft = 3.28084
ft_to_m = 0.3048


"""
printGPS [X]
    - does what you think

haversine [X]
    -dist from two gps locations

getDist [X]
    - returns dist of any two x,y coordinates

get_location_metres [X]
    -given GPS, dNorth (m), dEast (m) returns a new gps location

get_bearing [X]:
    -given two GPS, calculates bearing from the east relative to the first point

calcGPS_from_map [_]:
    -given origin and bearing of field, and the abs_ft map pos, calculate global gps

"""


def printGPS(gps):
    print"GPS:[{},{}]".format(gps[0],gps[1])
def getDist(pt1,pt2):
	return sqrt((pt1[1]-pt2[1])**2.0 + (pt1[0]-pt2[0])**2.0)
	
def haversine(gps1,gps2):
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [gps1[1], gps1[0], gps2[1], gps2[0]])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2.0)**2.0 + cos(lat1) * cos(lat2) * sin(dlon/2)**2.0
    c = 2.0 * asin(sqrt(a)) 
    r = 6378137.0 # Radius of earth in kilometers. Use 3956 for miles
    return c * r
    
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius * cos(pi*original_location[0]/180.0))

    #New position in decimal degrees
    newlat = original_location[0] + (dLat * 180.0/pi)
    newlon = original_location[1] + (dLon * 180.0/pi)
    return [newlat,newlon, original_location[2]]



"""
#get_bearing tests
#############################################################
gps_stl = LocationGlobal(33.648045, -117.849542,0)
gps_wtl = LocationGlobal(33.649771, -117.850943,0)
gps_etl = LocationGlobal(33.649794, -117.848202,0)
print"pt 1 towards a direct south point: {}".format(get_bearing(gps_tl,gps_stl))
print"pt 1 towards a direct west point: {}".format(get_bearing(gps_tl,gps_wtl))
print"pt 1 towards a direct east point: {}".format(get_bearing(gps_tl,gps_etl))
***
pt 1 towards a direct south point: 269.93294817
pt 1 towards a direct west point: 179.305787159
pt 1 towards a direct east point: 1.71236831961
"""

def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2[0] - aLocation1[0]
    off_y = aLocation2[1] - aLocation1[1]
    bearing = 90.0 + atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def calcGPS_from_map(origin, field_bearing, map_pos):
    map_point_bearing = atan2(map_pos[1],map_pos[0])*57.2958
    map_point_dist = sqrt(map_pos[0]**2 + map_pos[1]**2) * ft_to_m

    abs_point_bearing = float(field_bearing + 270.0 + map_point_bearing)*deg_to_rad


    dN = sin(abs_point_bearing) * map_point_dist
    dE = cos(abs_point_bearing) * map_point_dist

    return get_location_metres(origin,dN,dE)