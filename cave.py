import matplotlib
import warnings
import matplotlib.pyplot as plt
import pickle
import random
import numpy as np
import copy
from constants import *

warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

#generate caves using basic automata algorithm
def caveAutomata(cave, b, s, num):
	for _ in range(num):
		tempcave = copy.copy(cave)
		for x in range(CAVEDIM):
			for y in range(CAVEDIM):
				numNeighbors = countNeighbors(tempcave, x, y)
				if tempcave[x][y] > MIDDLE_BOUND: 
					if numNeighbors not in s:
						cave[x][y] = WALL
					else:
						cave[x][y] = GROUND
				else:
					if numNeighbors not in b:
						cave[x][y] = WALL
					else:
						cave[x][y] = GROUND

#for cave automata generation
def countNeighbors(cave, x, y):
	count = 0
	for i in range(-1, 2):
		for j in range(-1, 2):
			neighbor_x = x+i
			neighbor_y = y+j
			if i is 0 and j is 0:
				pass
			elif neighbor_x < 0 or neighbor_y < 0 \
				or neighbor_x >= CAVEDIM or neighbor_y >= CAVEDIM:
				count += 1
			elif cave[neighbor_x][neighbor_y] >=0:
				count += 1
	return count

#closes in walls to make the world closed off
def blackenEdges(cave):
	for i in range(CAVEDIM):
		for j in range(EDGE_THICKNESS):
			cave[i][j] = WALL
			cave[i][CAVEDIM-j-1] = WALL
	for i in range(CAVEDIM):
		for j in range(EDGE_THICKNESS):
			cave[j][i] = WALL
			cave[CAVEDIM-j-1][i] = WALL

def createEntrance(cave):
	for i in range(BOX_SIZE):
		for j in range(BOX_SIZE):
			cave[i + EDGE_THICKNESS][j + CAVEDIM - BOX_SIZE + EDGE_THICKNESS]


def collapseCaveRandom(cave):
	collapsePoint = (random.randint(0, CAVEDIM-1), random.randint(0, CAVEDIM-1))
	for i in range(-COLLAPSE_CONSTANT, COLLAPSE_CONSTANT+1):
		if collapsePoint[0]+i >= 0 and collapsePoint[0]+i < CAVEDIM:
			for j in range(-COLLAPSE_CONSTANT, COLLAPSE_CONSTANT+1):
				if collapsePoint[1]+j >= 0 and collapsePoint[1]+j < CAVEDIM:
					if random.random()>.35:
						cave[collapsePoint[0]+i][collapsePoint[1]+j] = WALL



def updateCave(cave):
	collapseCaveRandom(cave)

def drawCave():
	##########DRAW CAVE##################
	# make values from -5 to 5, for this example
	cave = np.random.rand(CAVEDIM,CAVEDIM)*10-BALANCE_CONSTANT

	blackenEdges(cave)
	caveAutomata(cave, B1, S1, 20) #create using rule 1
	caveAutomata(cave, B2, S2, 3) #updat using rule 2; smooths edges
	blackenEdges(cave)
	return cave

f = open('empty_cave.txt', 'wb')
cave = drawCave()

cmap = matplotlib.colors.ListedColormap(['gray','black', 'white','red', 'yellow'])
bounds=[UNKNOWN_BOUND, WALL_BOUND,MIDDLE_BOUND,GROUND_BOUND,8,10] #if 7, robot; if 9, target
norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)

img1 = plt.imshow(list(reversed(zip(*cave))), interpolation='nearest',
						cmap = cmap, norm=norm)

				
plt.ioff()
plt.pause(.1)
plt.plot()
plt.show()
print 'plotting?'
plt.pause(5)

pickle.dump(cave,f)