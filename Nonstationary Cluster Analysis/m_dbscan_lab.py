import pygame, sys, math, random, time, dbscan, constants
from pygame.locals import*
from dbscan import*
from constants import*

"""
***dbscan_obj.normalize ==> True to normalize points, False to see raw points
"""

#determines whether the DBSCAN lab opens files written in DATA_LIST_FILE_NAME or whether to pick individual data files
MULTIPLE_FILES = False
DATA_LIST_FILE_NAME = ["a5data_files.txt","DataFiles/List_Of_Files.txt", "DataFiles/OldDataFiles.txt"]
LIST_FILE_ID = 1
DATA_TEXT_FILE_NAME = ["DataFiles/points.csv","DataFiles/new_data/Control/Radius/CONTROLR_5B_3M_6S_QT_10_t1.txt", "DataFiles/test3.txt", "DataFiles/old_data/Control/Radius/CONTROLR_5B_3M_6S_QT_10_t1.txt"]
TEXT_FILE_ID = 0
DELIM = ["\t\t", " ", ","] 
DELIM_ID = 2 #**make sure you match delimiter with what files you're anaylzing

#Creates DataManager class that parses data files and stores them in as point objects
DATA = DataManager(DATA_LIST_FILE_NAME[LIST_FILE_ID])

#The DBSCAN class holds the DBSCAN algorithm
dbscan_obj = DBSCAN(t_eps = 80)
dbscan_obj.normalize = True #normalizes flash clusters into its actual position

#choose between initializing the DataManager with a single and series of files
if MULTIPLE_FILES:
    #RESET(): analyze(),connectClusters(),generateClusterRects()
    dbscan_obj.RESET(DATA.getPointsFromFile(delimiter = DELIM[DELIM_ID]))
else:
    dbscan_obj.RESET(DATA.getPointsFromFile(single_file = DATA_TEXT_FILE_NAME[TEXT_FILE_ID], delimiter = DELIM[DELIM_ID]))


#FlashClusterAnalysis class analyzes DBSCAN generated clusters and finds likely signatures of buckets
FCA = FCAnal()
FCA.initFCCs(dbscan_obj.clusters) #initialize FCA with clusters

#analyze() {findFCLines(): finds the signature of linear pathing of FCs, 
#           sortFCLines(): sorts each line by time
#           findFCVelocities(): calculates velocity of robot with distances between FC centroids and their average timestamps}
FCA.analyze()

#uses robot velocity to normalize point data
dbscan_obj.normalizePoints(FCA.getVelocities())

############################################################################
## ** GRAPHICAL INTERFACE **                                               #
############################################################################
##INITIALIZING PYGAME/WINDOW
pygame.init()
pygame.display.set_caption("DBSCAN")

screen = pygame.display.set_mode((WINDOW_SIZE[0],WINDOW_SIZE[1]))
grid_surface = pygame.Surface((WINDOW_SIZE[0],WINDOW_SIZE[1]))
annot_surface = pygame.Surface((WINDOW_SIZE[0],WINDOW_SIZE[1]))

fpsClock =  pygame.time.Clock()

DRAW_MAP_SIZE = [MAP_SIZE_MM[0]/MM_PXL[0],MAP_SIZE_MM[1]/MM_PXL[1]]
CELL_DRAW_SIZE = [CELL_SIZE_MM[0]/MM_PXL[0],CELL_SIZE_MM[1]/MM_PXL[1]]

# DRAWING FUNCTIONS
def drawGrid():
    for i in range(MAP_SIZE_MM[0]/CELL_SIZE_MM[0]):
        start_pt = (0,i*CELL_DRAW_SIZE[1])
        end_pt = (DRAW_MAP_SIZE[0],i*CELL_DRAW_SIZE[1])
        pygame.draw.line(grid_surface, (255,255,255), start_pt, end_pt, 1)

    for i in range(MAP_SIZE_MM[1]/CELL_SIZE_MM[1]):
        start_pt = (i*CELL_DRAW_SIZE[0],0)
        end_pt = (i*CELL_DRAW_SIZE[0],DRAW_MAP_SIZE[1])
        pygame.draw.line(grid_surface, (255,255,255), start_pt, end_pt, 1)

    grid_surface.set_alpha(30)
    screen.blit(grid_surface,(0,0))

def drawPoint(pos, color = (255,0,0)):
    pygame.draw.circle(screen, color, pos, POINT_DRAW_SIZE)

def drawRect(rect, color = (255,255,255)):
    pygame.draw.rect(annot_surface, color, rect, RECT_STROKE_WIDTH)
    annot_surface.set_alpha(100)
    screen.blit(annot_surface,(0,0))

def refreshSurfaces():
        screen.fill((0,0,0))
        grid_surface.fill((1,1,1))
        grid_surface.set_colorkey((1,1,1))
        annot_surface.fill((1,1,1))
        annot_surface.set_colorkey((1,1,1))
def waypointGen():
    direction = 0 #0-up 1-right 2-down 3-left
    map_length = 200
    unit = 12.0
    range_to_change =  (map_length/unit)-2 #in units
    range_travelled = 0 #in units
    waypoints = [[0,0]]
    directions = ["90"]
    distance_travelled = 0
    while(range_to_change > 0):

        for i in range(2):
            while(range_travelled < range_to_change):
                if(direction == 0):
                    #up (+)y
                    waypoints.append([waypoints[-1][0],waypoints[-1][1]+(2*unit)])
                    distance_travelled += (2*unit)
                elif(direction == 1):
                    #right (+)x
                    waypoints.append([waypoints[-1][0]+(2*unit),waypoints[-1][1]])
                elif(direction == 2):
                    #down (-)y
                    waypoints.append([waypoints[-1][0],waypoints[-1][1]-(2*unit)])
                elif(direction == 3):
                    #left (-)x
                    waypoints.append([waypoints[-1][0]-(2*unit),waypoints[-1][1]])
                

                range_travelled += 2
                """
                if(direction == 0):
                    directions.append("90")
                elif(direction == 1):
                    directions.append("0")
                elif(direction == 2):
                    directions.append("270")
                elif(direction == 3):
                    directions.append("180")
                """
            direction += 1
            direction %= 4
            
            range_travelled = 0
            print(waypoints[-1])

        range_to_change -= 2
    return [waypoints,directions]


waypoints = waypointGen()
print(waypoints)
############################################################################

while True:
    refreshSurfaces()
    drawGrid()
    
    #*** DRAWING CLUSTER RECTS ***#
    for rect in dbscan_obj.cluster_rects:
        drawRect(rect[0], rect[1])
    #*****************************#

    #*** DRAWING POINTS ***#
    for pt in dbscan_obj.noise:
        drawPoint([int(pt.coord[0]/MM_PXL[0]),int(pt.coord[1]/MM_PXL[1])], (50,50,50))

    for c in dbscan_obj.clusters:
        for pt in c.points:
            n_c = [255,0,0]
            if pt.classification == "CORE":
                n_c[2] = 255
            drawPoint([int(pt.coord[0]/MM_PXL[0]),int(pt.coord[1]/MM_PXL[1])], n_c)
    #**********************#
    
    for i in range(len(waypoints[0])):
        drawPoint([int(waypoints[0][i][0])*2,int(waypoints[0][i][1])*2], (255,255,255))

    if pygame.mouse.get_pressed() == (1,0,0):
        m_pos = pygame.mouse.get_pos()
        dist = math.sqrt((8000 - (m_pos[0]*20.0))**2 + (8000 - (m_pos[1]*20.0))**2)
        print(dist)
    keys = pygame.key.get_pressed()
    if keys[K_SPACE]:
        change_val = 5
    else:
        change_val = 1

    for event in pygame.event.get():
        if event.type == QUIT:
                pygame.quit()
                sys.exit()

        if event.type == KEYDOWN:
            #[: previous file
            #]: next file
            if event.key == K_LEFTBRACKET:
                if MULTIPLE_FILES:
                    dbscan_obj.RESET(DATA.getPointsFromFile(-1, DELIM[DELIM_ID]))
                    FCA = FCAnal()
                    FCA.initFCCs(dbscan_obj.clusters)
                    FCA.analyze()
                    dbscan_obj.normalizePoints(FCA.getVelocities())
            elif event.key == K_RIGHTBRACKET:
                if MULTIPLE_FILES:
                    dbscan_obj.RESET(DATA.getPointsFromFile(1, DELIM[DELIM_ID]))
                    FCA.initFCCs(dbscan_obj.clusters)
                    FCA.analyze()
                    dbscan_obj.normalizePoints(FCA.getVelocities())
            #up/down arrow: changes eps value
            #left/right arrow: changes minPts value
            if event.key == K_UP:
                dbscan_obj.changeDEps(change_val)
                dbscan_obj.RESET()
            elif event.key == K_DOWN:
                dbscan_obj.changeDEps(-1*change_val)
                dbscan_obj.RESET()
            elif event.key == K_LEFT:
                dbscan_obj.changeMinPts(-1*change_val)
                dbscan_obj.RESET()
            elif event.key == K_RIGHT:
                dbscan_obj.changeMinPts(1*change_val)
                dbscan_obj.RESET()
            elif event.key == K_PLUS:
                dbscan_obj.changeTEps(-1*change_val)
                dbscan_obj.RESET()
            elif event.key == K_MINUS:
                dbscan_obj.changeTEps(change_val)
                dbscan_obj.RESET()

    pygame.display.update()
    fpsClock.tick(60)