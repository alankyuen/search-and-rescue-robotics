from cluster_test import*
from dbscan_test import*
from constants_test import*
from field_test import*
from math import radians
import utm
################################################################################################
"""
GOALS:
Saturday [5/13] Testable SARA
Monday Night[5/8] SARA can move to a waypoint, and non-router methods (ad-hoc network)
Wednesday Night[5/10] SARA has navigation capabilities (spiral and manually-placed)
Friday Night [5/12] SARA fully autonomous, navigating a path and recording lidar data.


TASK LIST:

-Goto()
    1) does anybody know any possible ways? mission planner?
    2) what does the current simple_goto() do now? is there a way to view the innards of PH?
    3) before we commit to any decision to an attempt to fix it, how long will it take? anything possibly faster?

-WP Path:
    1) does the waypoint generator work?
    2) just in case the robot's compass is not very accurate and the waypoint generator will produce a path that is dangerous for SARA
        can you have a backup system for manual waypoint inputs

-mobile hotspot to replace router:
    1) is it possible?

-system testing
    1)do all of the functions that we've written work?
    2)can we test a series of points to see if it flows correctly?
    3)can we get good testing data quickly?
"""
################################################################################################


"""
top left: 33.649754, -117.849540
bottom left: 33.648467, -117.850491
bottom right: 33.647679, -117.849021 
top right: 33.649085, -117.848014
gps_tl = [(33.649754, -117.849540, 0)
gps_bl = [(33.648467, -117.850491, 0)
gps_br = [(33.647679, -117.849021, 0)
gps_tr = [(33.649085, -117.848014, 0)


#point(ts, gps_reading, [lidar heading from robot, dist, q], robot heading from field)
p1 = point(1500,gps_tl,[30, 60000, 25], 90)
p2 = point(3000,gps_tl,[30,5000,25],90)
field_bearing = get_bearing(gps_tl, gps_bl)
p1.calculateAbsPos(gps_tl, field_bearing)
p1.printPt(calculated = True)
"""
"""
gps_tl = [(33.649897, -117.848503, 0)
gps_bl = [(33.649815, -117.848561, 0)
gps_br = [(33.649703, -117.848345, 0)
gps_tr = [(33.649785, -117.848283, 0)
"""
"""
class "field":
    __init__:
        const vars: field_dimensions_ft/m, effective_sensor_range_ft/m, grid_dimensions(ft/m)
        data struct: cells[r][c]
        calculated vars: field_bearing, gps_waypoints, ft_waypoints
        dbscan vars: buckets_gps

    waypointGen [_]
        variables: field_width, S_R
        methods used: calcGPS_from_map

    addPoint [_]
        adds point to appropriate cell
        returns cell index that it has the point inserted into

    getPaddedCell [_]
        returns points of cell and neighboring points for DBSCAN

"""

"""
gps_tl = [33.649897, -117.848503, 0]
gps_bl = [33.649815, -117.848561, 0]
gps_br = [33.649703, -117.848345, 0]
gps_tr = [33.649785, -117.848283, 0]

gps_tl2 = [33.649754, -117.849540,0]
gps_bl2 = [33.648467, -117.850491,0]
gps_br2 = [33.647679, -117.849021,0]
gps_tr2 = [33.649085, -117.848014,0]

f1 = field()
for wps in f1.waypointGen(gps_tr2, get_bearing(gps_tr2, gps_br2)):
    printGPS(wps)
print(f1.ft_waypoints)
GPS:[33.6491237068,-117.848078922]
GPS:[33.648732545,-117.848415464]
GPS:[33.6489739921,-117.848820436]
GPS:[33.64931111,-117.848530391]
GPS:[33.6491470765,-117.848255263]
GPS:[33.6489180466,-117.848452313]
GPS:[33.6490046664,-117.848597597]
GPS:[33.6491256083,-117.848493543]
GPS:[33.6491164022,-117.848478102]
GPS:[33.6491035482,-117.848489161]
[[24.2782, 0], [24.2782, 175.7218], [175.7218, 175.7218], [175.7218, 24.2782], [72.8346, 24.2782], [72.8346, 127.1654], [127.1654, 127.1654], [127.1654, 72.8346], [121.39099999999999, 72.8346], [121.39099999999999, 78.60900000000001]]
"""

"""

def addPoint(self, pt):
    #pt: .origin, .raw, .abs, .visited, .cluster_name
    #add point to its cell
    row = min(max(0,int(pt.abs_ft[1]/self.effective_sensor_range_ft)), self.grid_dimensions[1]-1)
    col = min(max(0,int(pt.abs_ft[0]/self.effective_sensor_range_ft)), self.grid_dimensions[0]-1)

    self.cells[row][col].append(pt)
    return [row,col]
"""
#soccer field
gps_tl = [33.649754, -117.849540,0]
gps_bl = [33.648467, -117.850491,0]
gps_br = [33.647679, -117.849021,0]
gps_tr = [33.649085, -117.848014,0]
gps_br2 = [33.648668, -117.848291,0]

f1 = field()
f2 = field()


bearing1 = get_bearing(gps_bl, gps_tl)
gps_bl_shifted = get_location_metres(gps_bl, cos(radians(bearing1+90)) * f1.effective_sensor_range_ft/2.0, sin(radians(bearing1+90)) * f1.effective_sensor_range_ft/2.0)

f1.waypointGen(gps_bl, bearing1)
f2.waypointGen(gps_bl_shifted,bearing1)

f1.gps_waypoints += f2.gps_waypoints[::-1]

f1.ft_waypoints += f2.ft_waypoints[::-1]

for wps in f1.gps_waypoints:
    printGPS(wps)

"""
gps1 = [33.649754, -117.849540,0]
north_of_gps1 = [33.643945, -117.825836,0]
gps2 = [33.649235, -117.849940,0]

bearing = radians(get_bearing(gps1,gps2))

d = haversine(gps1,gps2)

gps2_pseudo = get_location_metres(gps1,cos(bearing)*d,sin(bearing)*d)
print"pseudo: dist:{} bearing:{}".format(haversine(gps1,gps2_pseudo),get_bearing(gps1,gps2_pseudo))
print"true: dist:{} bearing:{}".format(haversine(gps1,gps2),get_bearing(gps1,gps2))

#224.421949235-229.653351555
#335.05678748-330.808543212
"""
#self, ts = 0, gps_at_measurement = [], raw_measurement = [], robotBearing = 0, abs_position = []
p1 = point(1500,gps_bl,[90, 5000, 25], 90)
p2 = point(3000,gps_bl,[90,5000,25],90)
field_bearing = get_bearing(gps_bl, gps_tl)
print(field_bearing)
p1.calculateAbsPos(gps_bl, field_bearing)
p1.printPt(calculated = True)
