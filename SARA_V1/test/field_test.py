from constants_test import *
"""
class "field":
    __init__:
        const vars: field_dimensions_ft/m, effective_sensor_range_ft/m, grid_dimensions(ft/m)
        data struct: cells[r][c]
        calculated vars: field_bearing, gps_waypoints, ft_waypoints
        dbscan vars: buckets_gps

    waypointGen [_]
        variables: field_width, S_R

    addPoint [_]
        adds point to appropriate cell
        returns cell index that it has the point inserted into

    getPaddedCell [_]
        returns points of cell and neighboring points for DBSCAN

"""
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

    def waypointGen(self, origin, field_bearing):
        field_width = 200.0
        field_width_half = field_width/2.0
        S_R = self.effective_sensor_range_ft

        self.gps_waypoints = []
        self.ft_waypoints = []

        waypoint = [S_R,0]
        self.ft_waypoints.append(waypoint)
        self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing,waypoint))
        #print(waypoint)    

        delta_wp = [0,field_width-S_R] #(+)

        waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
        self.ft_waypoints.append(waypoint)
        self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing, waypoint))
        #print(waypoint)
        negate = 1
        i = 1
        while(abs(field_width_half-waypoint[0]) > S_R and abs(field_width_half-waypoint[1]) > S_R):
            delta_wp = [negate*(field_width-(2*i*S_R)),0] #(+)
            waypoint = [waypoint[0] + delta_wp[0],waypoint[1] + delta_wp[1]]
            self.ft_waypoints.append(waypoint)
            self.gps_waypoints.append(calcGPS_from_map(origin,field_bearing,waypoint))
            #print(waypoint)

            if(abs(field_width_half-waypoint[0]) > S_R or abs(field_width_half-waypoint[1]) > S_R):
                delta_wp = [0,negate*((2*i*S_R)-field_width)] #(+)
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
        row = min(max(0,int(pt.abs_ft[1]/self.effective_sensor_range_ft)), self.grid_dimensions[1]-1)
        col = min(max(0,int(pt.abs_ft[0]/self.effective_sensor_range_ft)), self.grid_dimensions[0]-1)

        self.cells[row][col].append(pt)
        return [row,col]

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