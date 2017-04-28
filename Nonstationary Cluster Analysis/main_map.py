import pygame, sys, math, random, time, dbscan, constants2, serial
from pygame.locals import*
from dbscan import*
from constants2 import*

def getCellCoord(pos, cell_size):
    return [int(pos[0]/cell_size), int(pos[1]/cell_size)]

class point:
    def __init__(self, r_pos, ts, q):
        self.abs_pos = [r_pos[0]+30480,r_pos[1]+30480]
        self.rel_pos = r_pos
        self.gps = [0,0]

        self.timestamp = ts
        self.quality = q

ser = serial.Serial ('/dev/cu.usbmodem1411', 115200)

main_map = [[[] for row in range(map_dimensions[1])] for col in range(map_dimensions[0])]

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

############################################################################
refreshSurfaces()
drawGrid()
while True:
        

    line = ser.readline()
    data = line.split(",")#words: [ms, mm, deg, quality]
    if(len(data) != 4):
        continue
    try:
        timestamp = int(data[0])
        dist = float(data[1])
        angle = float(data[2])
        quality = int(data[3])
    except ValueError:
        continue
        
    temp_pos = [dist * math.cos(angle*DegToPi),float(dist) * math.sin(float(angle)*DegToPi)]
    point_cell_coord = getCellCoord(temp_pos, CELL_SIZE_MM[0])
    print "pos:{} timestamp: {} quality: {}".format(temp_pos, timestamp, quality)
    pt = point(temp_pos, timestamp, quality)

    main_map[point_cell_coord[0]][point_cell_coord[1]].append(pt)
    draw_coord = [int(pt.abs_pos[0]/MM_PXL[0]),int(pt.abs_pos[1]/MM_PXL[1])]
    print(draw_coord)
    drawPoint(draw_coord, (255,255,255))

    #**********************#

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
            
    pygame.display.update()
    fpsClock.tick(60)