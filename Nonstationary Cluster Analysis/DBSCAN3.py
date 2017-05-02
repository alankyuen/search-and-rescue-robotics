import math
from math import radians, cos, sin, asin, sqrt

def haversine(gps1,gps2):
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [gps1[1], gps1[0], gps2[1], gps2[0]])

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
    newlat = original_location[0] + (dLat * 180/math.pi)
    newlon = original_location[1] + (dLon * 180/math.pi)
    return [newlat,newlon]

def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2[0] - aLocation1[0]
    off_y = aLocation2[1] - aLocation1[1]
    bearing = 90 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00

    return bearing

def calcGPS_from_map(origin, field_bearing, map_pos):
    map_point_bearing = math.atan2(map_pos[1],map_pos[0])*57.2958
    map_point_dist = math.sqrt(map_pos[0]**2 + map_pos[1]**2) * 0.3048

    print"pt_bearing:{} && pt_dist:{}".format(map_point_bearing,map_point_dist)
    abs_point_bearing = field_bearing + 270 + map_point_bearing

    dN = math.sin(abs_point_bearing*0.0174533) * map_point_dist
    dE = math.cos(abs_point_bearing*0.0174533) * map_point_dist

    return get_location_metres(origin,dN,dE)

class point:
    def __init__(self, ts = 0, gps_origin = [], raw_measurement = [], abs_position = []):
        #raw_measurement = [bearing(degree), distance(mm), quality(0-60)]
        self.timestamp = ts
        self.origin = gps_origin
        self.raw = raw_measurement
        self.abs_ft = abs_position #on 200ft by 200ft map
        self.abs_gps = []

        self.visited = False
        self.classification = ""
        self.cluster_name = ""

    def calculateAbsPos(self, gps_start, field_bearing, robot_bearing):
        #dN = sin(theta + phi - 90)*r                          
        #dE = cos(theta + phi - 90)*r
        #(lidar_bearing) self.raw[0]: relative to the robot's east bearing [Degrees](left +, right -)
        #robot_bearing: relative to field's east bearing [Degrees](left +, right -)
        #field_bearing: relative to world's east bearing [Degrees](left +, right -)
        abs_bearing = (180 + field_bearing + robot_bearing + self.raw[0])%360
        #print"abs_bearing:{}".format(abs_bearing) #abs_bearing of point from EAST [WORKS]
        dN = math.sin(abs_bearing*0.0174533) * self.raw[1] /1000.0
        dE = math.cos(abs_bearing*0.0174533) * self.raw[1] /1000.0

        self.abs_gps = get_location_metres(self.origin, dN, dE)
        print"abs_gps_of_pt:{}".format(self.abs_gps)# [WORKS]

        dist_meters = haversine(gps_start,self.abs_gps)
        rel_bearing = (450 - field_bearing + get_bearing(gps_start,self.abs_gps))%360
        #print"dist_meters:{} &&& rel_bearing:{}".format(dist_meters,rel_bearing)
        dN_map = math.sin(rel_bearing*0.0174533) * dist_meters
        dE_map = math.cos(rel_bearing*0.0174533) * dist_meters
        self.abs_ft = [dE_map*3.28084, dN_map*3.28084] 
        print(self.abs_ft)
    def getDistanceFrom(self, neighbor):
        return math.sqrt((self.abs_ft[0] - neighbor.abs_ft[0])**2 + (self.abs_ft[1] - neighbor.abs_ft[1])**2)
class cluster:
    def __init__(self, identity = 0):
        self.ID = identity
        self.points = []
        self.centroid_ft = []

    def addPoint(self, pt):
        self.points.append(point(pt.timestamp, pt.origin, pt.raw, pt.abs_ft))

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
class field:
   
    def __init__(self):
        self.field_dimensions_ft = [200.0,200.0]
        self.field_dimensions_m = [60.96,60.96]

        self.effective_sensor_range_ft = 12.1391 * 2
        self.effective_sensor_range_m = 3.7 * 2

        self.grid_dimensions = [int(self.field_dimensions_ft[0]/self.effective_sensor_range_ft),int(self.field_dimensions_ft[1]/self.effective_sensor_range_ft)]
        #self.grid_dimensions_m = [self.field_dimensions_m[0]/self.effective_sensor_range_m[0],self.field_dimensions_m[1]/self.effective_sensor_range_m[1]]
        self.cells = [[[None] for c in range(self.grid_dimensions[0])] for r in range(self.grid_dimensions[1])] #[r][c]

        self.field_bearing = 0
        self.buckets_gps = []

        self.gps_waypoints = []
        self.ft_waypoints = []

    def waypointGen(origin, field_bearing):
        S_R = self.effective_sensor_range_ft

        self.gps_waypoints = []
        self.ft_waypoints = []

        waypoint = [S_R,0]
        self.ft_waypoints.append(waypoint)
        self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing,waypoint))
        #print(waypoint)    

        delta_wp = [0,200-S_R] #(+)

        waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
        self.ft_waypoints.append(waypoint)
        self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing,waypoint))
        #print(waypoint)
        negate = 1
        i = 1
        while(abs(100-waypoint[0]) > S_R and abs(100-waypoint[1]) > S_R):
            delta_wp = [negate*(200-(2*i*S_R)),0] #(+)
            waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
            self.ft_waypoints.append(waypoint)
            self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing,waypoint))
            #print(waypoint)

            if(abs(100-waypoint[0]) > S_R or abs(100-waypoint[1]) > S_R):
                delta_wp = [0,negate*((2*i*S_R)-200)] #(+)
                waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
                self.ft_waypoints.append(waypoint)
                self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing,waypoint))
                #print(waypoint)
            negate *= -1
            i+= 1
        return self.gps_waypoints

    def addPoint(self, pt):
        #pt: .origin, .raw, .abs, .visited, .cluster_name
        #add point to its cell
        row = min(max(0,pt.abs_ft[1]/self.effective_sensor_range_ft), grid_dimensions[1])
        col = min(max(0,pt.abs_ft[0]/self.effective_sensor_range_ft), grid_dimensions[0])

        self.cells[row][col].append(pt)

    def getPaddedCell(self, coord, padding_ft = 3.2808):
        #3.2808 ft padding is an extra meter
        points = self.cells[coord[0]][coord[1]]
        #add all neighboring points
        if(coord[0] - 1 >= 0):
            y_bound = (coord[0]*self.effective_sensor_range_ft) - padding_ft
            for pt in self.cells[coord[0] - 1][coord[1]]:
                if(pt.abs[1] > y_bound):
                    points.append(pt)
            if(coord[1] - 1 >= 0):
                x_bound = (coord[1]*self.effective_sensor_range_ft) - padding_ft
                for pt in self.cells[coord[0] - 1][coord[1] - 1]:
                    if(pt.abs[1] > y_bound and pt.abs[0] > x_bound):
                        points.append(pt)
            if(coord[1] + 1 < len(self.grid_dimensions[1])):
                x_bound = ((coord[1] + 1) * self.effective_sensor_range_ft) + padding_ft
                for pt in self.cells[coord[0] - 1][coord[1] + 1]:
                    if(pt.abs[1] > y_bound and pt.abs[0] < x_bound):
                        points.append(pt)

        if(coord[0] + 1 < len(self.grid_dimensions[0])):
            y_bound = ((coord[0]+1)*self.effective_sensor_range_ft) + padding_ft
            for pt in self.cells[coord[0] + 1][coord[1]]:
                if(pt.abs[1] > y_bound):
                    points.append(pt)
            if(coord[1] - 1 >= 0):
                x_bound = (coord[1] * self.effective_sensor_range_ft) - padding_ft
                for pt in self.cells[coord[0] + 1][coord[1] - 1]:
                    if(pt.abs[1] > y_bound and pt.abs[0] > x_bound):
                        points.append(pt)
            if(coord[1] + 1 < len(self.grid_dimensions[1])):
                x_bound = ((coord[1] + 1) * self.effective_sensor_range_ft) + padding_ft
                for pt in self.cells[coord[0] + 1][coord[1] + 1]:
                    if(pt.abs[1] > y_bound and pt.abs[0] < x_bound):
                        points.append(pt)

        if(coord[1] - 1 >= 0):
            x_bound = (coord[1] * self.effective_sensor_range_ft) - padding_ft
            for pt in self.cells[coord[0]][coord[1] - 1]:
                if(pt.abs[0] > x_bound):
                    points.append(pt)
        if(coord[1] + 1 < len(self.grid_dimensions[1])):
            x_bound = ((coord[1] + 1) * self.effective_sensor_range_ft) + padding_ft
            for pt in self.cells[coord[0]][coord[1] - 1]:
                if(pt.abs[0] < x_bound):
                    points.append(pt)

        return points

#Density-based spatial clustering of applications with noise
class DBSCAN:
    #min points, density epsilon, time epsilon
    def __init__(self, mpts, d_eps, t_eps):
        self.minPts = mpts
        self.dist_epsilon = d_eps
        self.time_epsilon = t_eps

        self.time_dependent = True #cluster based on time
        self.space_dependent = True #cluster based on space

        self.clusters = []
        self.cluster_count = 0

        self.points = [] #points to scan
        self.noise = [] #noise placeholder

    def initPoints(self, arr_points):
        for p in arr_points:
            self.points.append(Point(p[0],p[1],p[2]))

    #returns cluster obj
    def findCluster(self, _name):
        for c in self.clusters:
            if(c.name == _name):
                return c
        return None

    #returns the list of points within eps distance
    def queryRegion(self, pt, time_dependent = True):
        neighbors = []
        for neighbor in self.points:
            if(neighbor == pt):
                continue
            if(time_dependent):
                if(pt.getDistanceFrom(neighbor) < self.dist_epsilon) and (abs(pt.timestamp - neighbor.timestamp) < self.time_epsilon):
                    neighbors.append(neighbor)
            else:
                if(pt.getDistanceFrom(neighbor) < self.dist_epsilon):
                    neighbors.append(neighbor)

        return neighbors

    #called on intial core points to expand the cluster
    def expandCluster(self, pt, neighbors, cluster, time_dependent = True):
        #adds p to cluster
        cluster.addPoint(pt)
        pt.cluster_name = cluster.name
        pt.inCluster = True

        #looks at neighbors for potential density-reachable core points
        for neighbor in neighbors:
            if not neighbor.visited:
                neighbor.visited = True

                n_neighbors = self.queryRegion(neighbor, time_dependent)
                if(len(n_neighbors) >= self.minPts):
                    #mark neighbor as core
                    #add the neighbor's neighbors onto the list of pending points to check
                    neighbor.classification = "CORE"
                    neighbors += n_neighbors
                else:
                    self.noise.append(neighbor)
                    neighbor.classification = "NOISE"
            
            if not neighbor.inCluster:
                #mark point in cluster
                cluster.addPoint(neighbor)
                neighbor.cluster_name = cluster.name
                neighbor.inCluster = True

        self.clusters.append(cluster)

    #uses queryRegion and expandCluster to identify dense regions of points as clusters
    def analyze(self, time_dependent = True):
        #A: look for core points
        for pt in self.points:
            if pt.visited:
                continue
            pt.visited = True

            neighbors = self.queryRegion(pt, time_dependent)
            if(len(neighbors) < self.minPts):
                #mark noise temporarily
                self.noise.append(pt)
                pt.classification = "NOISE"
            else:
                #mark core points
                c = Cluster("cluster_"+str(self.cluster_count))
                self.cluster_count += 1
                pt.classification = "CORE"
                self.expandCluster(pt, neighbors, c, time_dependent)

        #B: reiterate and look for reachable points in noise
        new_noise = []
        for pt in self.noise:
            for neighbor in self.queryRegion(pt, time_dependent):
                if(neighbor.classification == "CORE"):
                    #if this point is reachable, then add to the cluster
                    p.classification = "REACHABLE"
                    c = self.findCluster(neighbor.cluster_name)
                    c.addPoint(pt)
                    break
            if pt.classification == "NOISE":
                new_noise.append(pt)
        self.noise = new_noise

"""
#33.716318, -117.830465 top left corner 33.716278, -117.830047
#33.716039, -117.830054
c1 = [33.703317, -117.779587]
c2 = [33.704131, -117.779419]
c3 = [33.71648309670308, -117.83143788477567]
c4_pt = [33.703416, -117.779394]

field_bearing = get_bearing(c1,c2)
p1 = point(0, c1, [35,20964.8592505,15],[0,0])
p1.calculateAbsPos(c1, field_bearing, 90)
print(waypointGen(12.5,c1,field_bearing))
"""