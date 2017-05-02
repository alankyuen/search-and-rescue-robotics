from math import radians, cos, sin, asin, sqrt
from dronekit import LocationGlobal

deg_to_rad = 0.0174533
m_to_ft = 3.28084
ft_to_m = 0.3048

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
    
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location[0]/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat,newlon)

def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lat - aLocation1.lat
    off_y = aLocation2.lon - aLocation1.lon
    bearing = 90 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def calcGPS_from_map(origin, field_bearing, map_pos):
    map_point_bearing = math.atan2(map_pos[1],map_pos[0])*57.2958
    map_point_dist = math.sqrt(map_pos[0]**2 + map_pos[1]**2) * ft_to_m

    abs_point_bearing = (field_bearing + 270 + map_point_bearing)*deg_to_rad

    dN = math.sin(abs_point_bearing) * map_point_dist
    dE = math.cos(abs_point_bearing) * map_point_dist

    return get_location_metres(origin,dN,dE)