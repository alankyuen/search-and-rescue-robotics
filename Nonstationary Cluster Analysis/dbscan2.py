import sys, math, random, constants, copy, csv
from constants import*

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

############################################################################
#                                DBSCAN                                    #
############################################################################
# CLASSES:
#           -DataManager, Point, Cluster, DBSCAN, FCAnal
############################################################################

#Retrieves data from txt files and converts into x,y points
class DataManager:
    def __init__(self, data_directory_file):
        self.data_files = open(data_directory_file).read().splitlines()
        self.data_file_counter = 0

        self.points = []
        
        #filter low quality points
        self.MIN_QUALITY = 12

        #const used in polar-cartesian translations
        self.DEG_TO_PI = math.pi/180.0

    def changeDirectoryFile(self, data_directory_file):
        self.data_files = open(data_directory_file).read().splitlines()
        self.data_file_counter = 0
        del points[:]

    #returns points generated from data
    def getPointsFromFile(self, nextFile = 0, delimiter = " ", single_file = "", file_type = ".txt"):
        if single_file != "":
            file_name = single_file
            print("Extracting data from: " + file_name)
        else:
            if(self.data_file_counter + nextFile >= len(self.data_files) or self.data_file_counter + nextFile < 0):
                return None
            self.data_file_counter += nextFile
            file_name = self.data_files[self.data_file_counter]
            print("Extracting data from: " + file_name)


        del self.points[:]

        if(file_type == ".txt"):
            with open(file_name) as f:
                data = f.readlines()
        else:
            with open(file_name, 'rb') as csvfile:
                data = csv.reader(csvfile,delimiter='\n')
                print(data)

        for line in data:
            words = line.split(delimiter)#words: [ms, mm, deg, quality] #old: [mm,deg,quality]

            quality = float(words[3])
            dist = float(words[1])
            angle = float(words[2])
            timestamp = float(words[0])
            #if distance is less than half a meter or quality is lower than min, pass
            if quality < self.MIN_QUALITY:
                    continue

            #translates polar to cartesian
            x = float(dist)*math.cos(angle * self.DEG_TO_PI)
            y = float(dist)*math.sin(angle * self.DEG_TO_PI)
            
            #translate 6m from center to corner
            point = [x + (MAP_SIZE_MM[0]/2),y + (MAP_SIZE_MM[0]/2)] 
            self.points.append([point,quality,timestamp])

        return self.points

#holds points
class Point:
    def __init__(self, pos, qual, ts):
        #point data
        self.coord = pos
        self.quality = qual
        self.timestamp = ts

        #flags
        self.visited = False
        self.inCluster = False

        #id
        self.classification = "" #CORE, REACHABLE, NOISE
        self.cluster = "" #cluster name

    def getDistanceFrom(self, pos):
        return math.sqrt((self.coord[0] - pos[0])**2 + (self.coord[1] - pos[1])**2)

#holds points in groups ID'd by DBSCAN
class Cluster:
    def __init__(self, _name):
        self.name = _name #name/id of the cluster
        self.points = [] #holds points
        self.rect = []  #[x,y,w,h]
        self.centroid = [] #centroid of cluster
        self.avg_time_stamp = 0 #ms timestamp

    def addPoint(self, p):
        self.points.append(p)

    def getRect(self):
        #corner_points is rectangle coordinates: [min_x,min_y,max_x,max_y]
        corner_points = [self.points[0].coord[0],self.points[1].coord[1],self.points[0].coord[0],self.points[1].coord[1]] 
        for p in self.points[1:]:
            #finds top left corner
            if(p.coord[0] < corner_points[0]):
                corner_points[0] = p.coord[0]
            if(p.coord[1] < corner_points[1]):
                corner_points[1] = p.coord[1]
            #finds bottom right corner
            if(p.coord[0] > corner_points[2]):
                corner_points[2] = p.coord[0]
            if(p.coord[1] > corner_points[3]):
                corner_points[3] = p.coord[1]

        #if you want rect to be [x,y,w,h]
        corner_points[2] = corner_points[2] - corner_points[0]
        corner_points[3] = corner_points[3] - corner_points[1]

        self.rect = corner_points
        return self.rect

    def getDrawRect(self):
        draw_rect = [int(self.rect[0]/MM_PXL[0]),int(self.rect[1]/MM_PXL[1]),int(self.rect[2]/MM_PXL[0]),int(self.rect[3]/MM_PXL[1])]
        return draw_rect

    def getCentroid(self):
        self.getRect()
        self.centroid = [self.rect[0] + (self.rect[2]/2.0), self.rect[1] + (self.rect[3]/2.0)]
        return self.centroid

    def getAvgTimeStamp(self):
        self.avg_time_stamp = 0
        for p in self.points:
            self.avg_time_stamp += p.timestamp
        self.avg_time_stamp /= float(len(self.points))
        return self.avg_time_stamp

#Density-based spatial clustering of applications with noise
class DBSCAN:
    #min points, density epsilon, time epsilon
    def __init__(self, mpts = 2, d_eps = 150, t_eps = 80):
        self.minPts = mpts
        self.dist_epsilon = d_eps
        self.time_epsilon = t_eps

        self.time_dependent = True #cluster based on time
        self.space_dependent = True #cluster based on space

        self.clusters = []
        self.cluster_count = 0
        self.cluster_rects = [] #[rect,color]

        self.points = []
        self.noise = []

        self.normalize = True

    def initPoints(self, arr_points):
        for p in arr_points:
            self.points.append(Point(p[0],p[1],p[2]))

    #if you want to change the settings, call these before dbscan_RESET()
    def changeDEps(self, change_val):
        self.dist_epsilon += change_val
    def changeTEps(self, change_val):
        self.time_epsilon += change_val
    def changeMinPts(self, change_val):
        if(self.minPts + change_val > 0):
            self.minPts += change_val

    #resets DBSCAN for new analysis when you change data points or settings
    def RESET(self, points = None):
        del self.clusters[:]
        del self.noise[:]

        if(points):
            del self.points[:]
            self.initPoints(points)
        else:
            for p in self.points:
                p.visited = False
                p.inCluster = False
                p.classification = "" #CORE, REACHABLE, NOISE
                p.cluster = "" #cluster name

        self.minPts = 2
        self.dist_epsilon = 150
        self.analyze()
        self.connectClusters()
        self.generateClusterRects()
        print("num clusters - " + str(len(self.clusters)))
        print("D_eps: " + str(self.dist_epsilon) + " T_eps: " + str(self.time_epsilon) + " minPts: " + str(self.minPts))

    #returns cluster obj
    def findCluster(self, _name):
        for c in self.clusters:
            if(c.name == _name):
                return c
        return None

    #returns the list of points within eps distance
    def queryRegion(self, p, time_dependent = True):
        neighbors = []
        for neighbor in self.points:
            if(neighbor == p):
                continue
            if(time_dependent):
                if(p.getDistanceFrom(neighbor.coord) < self.dist_epsilon) and (abs(p.timestamp - neighbor.timestamp) < self.time_epsilon):
                    neighbors.append(neighbor)
            else:
                if(p.getDistanceFrom(neighbor.coord) < self.dist_epsilon):
                    neighbors.append(neighbor)

        return neighbors

    #called on intial core points to expand the cluster
    def expandCluster(self, p, neighbors, cluster, time_dependent = True):
        #adds p to cluster
        cluster.addPoint(p)
        p.cluster = cluster.name
        p.inCluster = True

        #looks at neighbors for potential density-reachable core points
        for neighbor in neighbors:
            if not neighbor.visited:
                neighbor.visited = True

                n_neighbors = self.queryRegion(neighbor, time_dependent)
                if(len(n_neighbors) > self.minPts):
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
                neighbor.cluster = cluster.name
                neighbor.inCluster = True

        self.clusters.append(cluster)

    #uses queryRegion and expandCluster to identify dense regions of points as clusters
    def analyze(self, time_dependent = True):
        #A: look for core points
        for p in self.points:
            if p.visited:
                continue
            p.visited = True

            neighbors = self.queryRegion(p, time_dependent)
            if(len(neighbors) < self.minPts):
                #mark noise temporarily
                self.noise.append(p)
                p.classification = "NOISE"
            else:
                #mark core points
                c = Cluster("cluster_"+str(self.cluster_count))
                self.cluster_count += 1
                p.classification = "CORE"
                self.expandCluster(p, neighbors, c, time_dependent)

        #B: reiterate and look for reachable points in noise
        new_noise = []
        for p in self.noise:
            for neighbor in self.queryRegion(p, time_dependent):
                if(neighbor.classification == "CORE"):
                    #if this point is reachable, then add to the cluster
                    p.classification = "REACHABLE"
                    c = self.findCluster(neighbor.cluster)
                    c.addPoint(p)
                    break
            if p.classification == "NOISE":
                new_noise.append(p)
        self.noise = new_noise

    def connectClusters(self, eps = 1):
        for c_i in self.clusters:
            c_i.getRect()
            for c_j in self.clusters:
                if c_i is c_j:
                    continue
                c_j.getRect()
                #calculates centroid of two clusters
                c_i_centroid= [c_i.rect[0]+(c_i.rect[2]/2),c_i.rect[1]+(c_i.rect[3]/2)]
                c_j_centroid = [c_j.rect[0]+(c_j.rect[2]/2),c_j.rect[1]+(c_j.rect[3]/2)]
                #gets distance between two clusters
                dist = math.sqrt((c_i_centroid[0] - c_j_centroid[0])**2 + (c_i_centroid[1] - c_j_centroid[1])**2)
                #if distance between the two clusters is less than threshold
                if(dist < eps):
                    #merge clusters
                    c_i.points += c_j.points
                    self.clusters.remove(c_j)
                    del c_j
        #print("new num clusters - " + str(len(self.clusters)))

    def generateClusterRects(self):
        del self.cluster_rects[:]

        cluster_colors = generateColors(len(self.clusters))
        for i in range(len(self.clusters)):
            self.clusters[i].getRect()
            self.cluster_rects.append([self.clusters[i].getDrawRect(),cluster_colors[i]])

        return self.cluster_rects

