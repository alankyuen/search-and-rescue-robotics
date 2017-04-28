import pygame, sys, math, random, time, constants
from pygame.locals import*
from constants import*

############################################################################
## ** GRAPHICAL INTERFACE **                                               #
############################################################################
##INITIALIZING PYGAME/WINDOW
pygame.init()
pygame.display.set_caption("VectorBasedClustering")

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

while True:
        refreshSurfaces()
        drawGrid()

        if pygame.mouse.get_pressed() == (1,0,0):
            m_pos = pygame.mouse.get_pos()
            dist = math.sqrt((8000 - (m_pos[0]*20.0))**2 + (8000 - (m_pos[1]*20.0))**2)
            print(dist)
        
        pygame.display.update()
        fpsClock.tick(60)