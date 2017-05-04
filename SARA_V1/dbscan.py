import cluster, math, constants
from cluster import *
#Density-based spatial clustering of applications with noise
class DBSCAN:
    #min points, density epsilon, time epsilon
    def __init__(self, mpts, d_eps, t_eps = 100):
        self.minPts = mpts
        self.dist_epsilon = d_eps
        self.time_epsilon = t_eps

        self.time_dependent = False #cluster based on time
        self.space_dependent = True #cluster based on space

        self.clusters = []
        self.cluster_count = 0

        self.points = None #points to scan
        self.noise = [] #noise placeholder

    def reset(self, arr_points):
        if not (self.points is None):
            del self.points
            self.points = []

        for p in arr_points:
            self.points.append(Point(p.timestamp,p.gps_reading,p.raw, p.robot_bearing, p.abs_ft))

    #returns cluster obj
    def findCluster(self, _name):
        for c in self.clusters:
            if(c.name == _name):
                return c
        return None

    #returns the list of points within eps distance
    def queryRegion(self, pt, time_dependent = False):
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
    def expandCluster(self, pt, neighbors, cluster, time_dependent = False):
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
    def analyze(self, time_dependent = False):
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

        #C:return list of centroids of clusters
        centroids = []
        for c in self.clusters:
            centroids.append(c.getCentroid())
        return centroids