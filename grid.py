import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import copy

CAVEDIM = 300
B1 = {6,7,8}
S1 = {3,4,5,6,7,8}

B2 = {5, 6, 7, 8}
S2 = {5, 6, 7, 8}

WALLBOUND = -6
MIDDLEBOUND = 0
GROUNDBOUND = 6
WALL = -1
GROUND = 1
BALANCECONSTANT = 4.6


def caveAutomata(zvals, b, s, num):
	for _ in range(num):
		tempZvals = copy.copy(zvals)
		for x in range(CAVEDIM):
			for y in range(CAVEDIM):
				numNeighbors = countNeighbors(tempZvals, x, y)
				if tempZvals[x][y] > MIDDLEBOUND: 
					if numNeighbors not in s:
						zvals[x][y] = WALL
					else:
						zvals[x][y] = GROUND
				else:
					if numNeighbors not in b:
						zvals[x][y] = WALL
					else:
						zvals[x][y] = GROUND


def countNeighbors(zvals, x, y):
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
			elif zvals[neighbor_x][neighbor_y] >=0:
				count += 1
	return count

#closes in walls to make the world closed off
def blackenEdges(zvals):
	edgeThickness = int(CAVEDIM/100)
	for i in range(CAVEDIM):
		for j in range(edgeThickness):
			zvals[i][j] = WALL
			zvals[i][CAVEDIM-j-1] = WALL
	for i in range(CAVEDIM):
		for j in range(edgeThickness):
			zvals[j][i] = WALL
			zvals[CAVEDIM-j-1][i] = WALL


def drawCave():
	# make values from -5 to 5, for this example
	zvals = np.random.rand(CAVEDIM,CAVEDIM)*10-BALANCECONSTANT
	
	blackenEdges(zvals)
	caveAutomata(zvals, B1, S1, 20) #create using rule 1
	caveAutomata(zvals, B2, S2, 3) #updat using rule 2; smooths edges


	# make a color map of fixed colors
	cmap = matplotlib.colors.ListedColormap(['black','white','red', 'yellow'])
	bounds=[WALLBOUND,MIDDLEBOUND,GROUNDBOUND,8,10] #if 7, robot; if 9, victim
	norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)

	# tell imshow about color map so that only set colors are used
	img = plt.imshow(zvals,interpolation='nearest',
	                    cmap = cmap,norm=norm)

	# make a color bar
	plt.colorbar(img,cmap=cmap,
	                norm=norm,boundaries=bounds,ticks=[-5,0,5,])

	plt.show()
	return zvals

# def collapseCaveRandom(cave):



def updateCave(cave):
	collapseCaveRandom(cave)

#run shit
cave = drawCave()
# updateCave(cave)
