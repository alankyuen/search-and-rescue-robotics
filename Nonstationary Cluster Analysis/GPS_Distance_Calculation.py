import math

def dist_between_GPS(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    feet = 6367 * c * 3280.84
    return feet


def delta_xy_to_gps(x, y, lat_field_center):
    lat_unit = 111300.0 * 3.28084 #feet
    long_unit = 111300.0 * 3.28084 * math.cos(lat_field_center) #feet

    gps_coord= (x/lat_unit, y/long_unit)

    return gps_coord

def delta_gps_to_xy(gps, gps_o):
    x = dist_between_GPS(gps[0], gps_o[1], gps_o[0], gps_o[1])
    y = dist_between_GPS(gps_o[0], gps[1], gps_o[0], gps_o[1])
    return (x,y)


gps_coord1 = (-117.76981383562088,33.73946121031605)
gps_coord2 = (-117.76990234851837,33.73934968777626)


print "Origin GPS: {} \nCurrent GPS: {} \nX,Y: {} Dist: {}".format(gps_coord1, gps_coord2, delta_gps_to_xy(gps_coord1,gps_coord2), dist_between_GPS(gps_coord1[0],gps_coord1[1],gps_coord2[0],gps_coord2[1]))

