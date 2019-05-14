import matplotlib
import matplotlib.pyplot as plt
from scipy.stats import truncnorm
import pickle
import math
import random
import numpy as np
import copy
import sys
import time
import threading
from constants import *

directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
# directions.append('Self')

#load the cave model
f = open(cave_file, 'rb')
cave = pickle.load(f)
sys.setrecursionlimit(1500)

def manhattanDistance(location_a, location_b):
	return abs(location_a[0] - location_b[0]) + abs(location_a[1] - location_b[1])

def getAdjacentDirections(direction):
	i = directions.index(direction)
	jIndex = (i - 1) % len(directions)
	kIndex = (i + 1) % len(directions)

	j = directions[jIndex]
	k = directions[kIndex]
	return [j, k]

def outOfBounds(location):
	(i, j) = location
	if i < EDGE_THICKNESS or i > CAVEDIM - EDGE_THICKNESS or \
	   j < EDGE_THICKNESS or j > CAVEDIM - EDGE_THICKNESS:
			return True
	return False

#some cave functions
def updateCaveLocation(location, value):
	(x, y) = location
	cave[x][y] = value

def getCaveTerrain(location):
	(x, y) = location
	return int(cave[x][y])

#swapl ground/wall in location. 
def fliplocation(locationValue):
	if locationValue is GROUND:
		return WALL
	return GROUND

#returnl nearest location in the given cardinal direction
def getLocationInDirection(d, location):
	location_new = list(location)
	if d is 'Self':
		return location
	if 'N' in d:
		location_new[1] += 1
	elif 'S' in d:
		location_new[1] -= 1
	if 'E' in d:
		location_new[0] += 1
	elif 'W' in d:
		location_new[0] -= 1
	return tuple(location_new)

#initialize noisy map with 30 percent accuracy
def generateNoisyExpectedMap():
	expectedMap = {}
	for i in range(CAVEDIM):
		for j in range(CAVEDIM):
			actualTerrain = cave[i][j]
			r = random.uniform(0, 1)
			if r < 1:
				terrain = actualTerrain
			# elif r < .03:
			# 	terrain = fliplocation(actualTerrain)
			else:
				terrain = UNKNOWN
			expectedMap[(i, j)] = terrain
	return expectedMap

#initialize countl based on the map generated from fn above
def generateMapCounts(terrainMap):
	mapCounts = {}
	for i in range(CAVEDIM):
		for j in range(CAVEDIM):
			mapCounts[(i, j)] = {}
			gcount = (terrainMap[(i, j)] == GROUND)*MAP_COUNT_INIT
			wcount = (terrainMap[(i, j)] == WALL)*MAP_COUNT_INIT
			mapCounts[(i, j)]['gcount'] = gcount
			mapCounts[(i, j)]['wcount'] = wcount
	return mapCounts


#the agent model
class Robot:
	def __init__(self):
		#the actual location, which the robot cannot see.
		self.location = ROBOT_START_LOCATION
		self.x = self.location[0]
		self.y = self.location[1]
		self.action = None
		#sensor data
		self.observations = {}
		self.collision = False
		
	def setlocation(self, newlocation):
		(x, y) = newlocation
		self.x = x
		self.y = y
		self.location = (self.x, self.y)
		# print self.location

	#actual move with random chance of failing
	def move(self, action):

		#detectl if a robot collidel with wall. No false positives. No false negatives. 
		def detectCollision(location_new):
			if getCaveTerrain(location_new) == WALL:
				return True
			else:
				return False
		successor_locations = getSurroundingPoints(self.location)
		adjacent_directions = getAdjacentDirections(action)
		weights = []
		possible_locations = []
		for d in adjacent_directions + [action]:
			l = successor_locations[d]
			weights.append(transitionProb(self.location, l, action))
			possible_locations.append(l)
		location_new = weightedPick(possible_locations, weights)
		self.collision = detectCollision(location_new)
		updateCaveLocation(self.location, GROUND) #vacate old location
		if not self.collision:
			self.setlocation(location_new) #update agent position if no collision
		updateCaveLocation(self.location, ROBOT) #new agent location

	#getl distance to target. noisy. returnl reading of closest target.
	def scanForTarget(self):
		noise = random.randint(-TARGET_SENSOR_NOISE, TARGET_SENSOR_NOISE)
		noisyDistance = manhattanDistance(TARGET_LOCATION, agent.location) + noise
		return noisyDistance < TARGET_SENSE_THRESH

	#actual observation with random chance of failing
	def observe(self):

		def observelocation(distantlocation):
			p = PROB_OBSERVATION_SUCCESS
			if getCaveTerrain(distantlocation) == TARGET:
				return TARGET #can observe goal without uncertainty if right next to it
			#otherwise, observe with a probability p of failure. 
			if random.uniform(0,1)<p:
				return getCaveTerrain(distantlocation)
			else:
				return getCaveTerrain(distantlocation)*-1

		observations = {}
		surrounding_locations = getSurroundingPoints(agent.location)
		for d in directions:
			l = surrounding_locations[d]
			observations[d] = observelocation(l)
		observations['collision'] = self.collision
		observations['targetNear'] = self.scanForTarget()
		# print 'real obs:', observations
		self.observations = observations

#not knowing where the target is, we just place it in a normally distributed location.
def generateTargetLocation(mapCounts):
	lower, upper = 0+EDGE_THICKNESS, CAVEDIM-EDGE_THICKNESS
	mu, sigma = CAVEDIM/2 + CAVEDIM/10, CAVEDIM/3 #mean is slightly skewed away from robot start location
	N = 2
	expectedMap = getMapFromCounts(mapCounts)
	while True:
		X = truncnorm((lower-mu)/sigma,(upper-mu)/sigma,loc=mu,scale=sigma) #this doesn't seem to work very well...
		samples = X.rvs(N)
		location = tuple([int(round(s)) for s in samples])
		# print location
		(x, y) = location
		if x > lower and y > lower and x < upper and y < upper and expectedMap[location] != WALL: #this is necessary because truncnorm behaves oddly
			return tuple(location)

def getMapFromCounts(mapCounts):
	expectedMap = {}
	for square in mapCounts:
		# print self.totalMapSum[square]
		totalCount = mapCounts[square]['gcount'] + mapCounts[square]['wcount']
		if mapCounts[square]['gcount'] > mapCounts[square]['wcount']:
			expectedMap[square] = GROUND
		elif mapCounts[square]['gcount'] < mapCounts[square]['wcount']:
			expectedMap[square] = WALL
		else:
			expectedMap[square] = UNKNOWN
	return expectedMap

#a sample of the robot's location and the current map. 
class Particle:
	def __init__(self, initialLocation, estimatedTargetLocation, initialMapCount):
		self.location = initialLocation
		self.targetLocation = estimatedTargetLocation
		self.mapCount = initialMapCount

	def getTerrain(self, location):
		gcount = self.mapCount[location]['gcount']
		wcount = self.mapCount[location]['wcount']
		if gcount + wcount != 0:
			groundRatio = float(gcount)/(gcount + wcount)
			r = random.uniform(0, 1)
			if r < groundRatio: #return ground/wall with ratio of observations. 
				return GROUND
			else:
				return WALL

		else: #if nothing observed in that square, return unknown.
			return UNKNOWN

	def shuffle(self):

		def getNearbyLocation(location):
			new_locations = getSurroundingPoints(location)
			possible_location = None
			for i in range(len(directions)):
				possible_location = random.choice(new_locations.values())
				if self.getTerrain(possible_location) != WALL:
					break
			return possible_location

		self.location = getNearbyLocation(self.location)
		self.targetLocation = getNearbyLocation(self.targetLocation)

	def updateInternalMap(self, observation):
		def updateMapSquare(location, o):
			if location not in self.mapCount: #init
				self.mapCount[location] = {}
				self.mapCount[location]['gcount'] = 0
				self.mapCount[location]['wcount'] = 0
			if o is GROUND:
				self.mapCount[location]['gcount'] += 1
			elif o is WALL:
				self.mapCount[location]['wcount'] += 1

		surrounding_locations = getSurroundingPoints(self.location)
		for d in directions:
			o = observation[d]
			# updateMapSquare(surrounding_locations[d], o)

#should each belief represent a particle with itl own internal map as per that one paper?
#or should particles just be the location of the robot? Ugh. 
class Belief:
	def __init__(self):
		self.expectedMap = generateNoisyExpectedMap()
		self.totalMapSum = generateMapCounts(self.expectedMap) #generateTargetLocation(self.totalMapSum)
		self.particles = [Particle(ROBOT_START_LOCATION, TARGET_LOCATION, self.totalMapSum) for _ in range(NUMBER_PARTICLES)] #one particle; start location. 
		self.expectedPosition = ROBOT_START_LOCATION
		# self.estimatedLocationNearestTarget = 
		self.uncertainty = 0
		self.history = []
		print 'belief initialized'

	def sampleParticle(self):
		return random.choice(self.particles)

	def shuffleParticles(self):
		for p in self.particles:
			p.shuffle()

	#updatel particles based on real action and observation. particle filter without weights. 
	def updateParticles(self, action, observation):
		# print observation
		b_new = []
		for i in range(NUMBER_PARTICLES):
			p = None
			location = tuple()
			while len(b_new) < NUMBER_PARTICLES: #sample particles until they are not shit. 
				state = self.sampleParticle()
				location = state.location
				# print len(b_new)
				if self.expectedMap[location] == WALL or outOfBounds(location):
					continue

				for _ in range(NUMBER_PARTICLES): #give the simulation 200 trials before resampling. 
					state_new, o_new, _ = blackBoxSimulator(state, action)
					# print location, s_new, action
					o_new_string, o_string = getObservationStringFromDict(o_new), getObservationStringFromDict(observation)
					# print o_new_string, o_string
					if o_new_string == o_string and not outOfBounds(state_new.location):
						state_new.updateInternalMap(observation)
						b_new.append(state_new)
						break
				state.shuffle() #if particle fails after simulation, shuffle it. 
		self.particles = b_new

	def calculateExpectedMap(self):

		def getTranslatedlocation(expectedLocation, particleLocation, pMaplocation):
			xDiff = expectedLocation[0] - particleLocation[0]
			yDiff = expectedLocation[1] - particleLocation[1]
			translatedlocation = tuple([pMaplocation[0]+xDiff, pMaplocation[1]+yDiff])
			return translatedlocation
		
		for p in self.particles:
			for location in p.mapCount:
				translatedlocation = getTranslatedlocation(self.expectedPosition, p.location, location)
				if outOfBounds(translatedlocation):
					continue
				for tile in p.mapCount[location]:
					self.totalMapSum[translatedlocation][tile] += float(p.mapCount[location][tile])/NUMBER_PARTICLES
		
		self.expectedMap = getMapFromCounts(self.totalMapSum)

	def calculateExpectedLocation(self):
		total = [0, 0]
		for p in self.particles:
			total = map(sum,zip(total,p.location))
		average = map(lambda x: int(float(x)/NUMBER_PARTICLES), total)
		self.expectedPosition = average

	def calculateUncertaintySD(self):
		SD = math.sqrt(sum((self.expectedPosition[0] - p.location[0])**2 
					+ (self.expectedPosition[1] - p.location[1])**2 for p in self.particles))
		self.uncertainty = SD

	# def estimateLocationNearestTarget(self): #wip

	def getEstimatedMapPlot(self):
		mapArray = []
		for i in range(CAVEDIM):
			mapArray.append([])
			for j in range(CAVEDIM):
				mapArray[i].append(self.expectedMap[(i, j)])
		return list(reversed(zip(*mapArray)))

	def updateActualHistory(self, action, observation):
		self.history.append(action)
		observationStr = getObservationStringFromDict(observation)
		self.history.append(observationStr)

	def updateBelief(self, action, observation):
		self.updateParticles(action, observation)
		self.updateActualHistory(action, observation)
		self.calculateExpectedLocation()
		# self.calculateExpectedMap()
		# self.estimateLocationNearestTarget()
		# self.calculateUncertaintySD()
		# print 'uncertainty:', self.uncertainty

#create the agent that will be navigating the cave. 
#in thil case, we are going with one agent. 
agent = Robot()
#create the belief space with all of the particles, the expected map, etc. 
#(map might not work; I need to test thil shit.)
belief = Belief()


#returnl euclidian distance between two points
def distance(location_a, location_b):
	return int(round(math.sqrt((location_a[0] - location_b[0])**2 + (location_a[1] - location_b[1])**2)))


#from the given location, calculate the coordinatel of the new location. if the new location
#il a wall instead of floor, return the given location rather than a new location. 
def getNewLocation(d, location):
	location_new = getLocationInDirection(d, location)
	# print d
	if cave[location_new[0]][location_new[1]] == WALL:
		return location
	return location_new

#calculate the new locations for each direction with the environment factored in
def getSuccessorLocations(location):
	successor_locations = {}
	for d in directions:
		successor_locations[d] = getNewLocation(d, location)
	return successor_locations

#return location coordinatel agnostic of whether or not there is a wall in the location of
#the location. 
def getSurroundingPoints(location):
	surrounding_locations = {}
	for d in directions:
		l = getLocationInDirection(d, location)
		if not outOfBounds(l):
			surrounding_locations[d] = l
		else:
			surrounding_locations[d] = location
	return surrounding_locations

#calculate the transistion probability given the previous location, the new location, and the action
#thil is only used for the actual agent movement. We do not use a model of it for calculating belief. 
def transitionProb(location, location_new, action):
	possibleDirections = getAdjacentDirections(action)
	successor_locations = [getLocationInDirection(d, location) for d in possibleDirections]
	attempted_location = getNewLocation(action, location)
	p = PROB_MOVEMENT_SUCCESS
	if attempted_location == location_new:
		return p
	elif location_new in successor_locations:
		return (1-p)/len(successor_locations)
	else:
		return 0


def weightedPick(locations, weights):
	r = random.uniform(0, sum(weights))
	l = 0.0
	for i in range(len(locations)):
		l += weights[i]
		if r<l: return locations[i]
	return locations[-1]

def getlocationDirection(location, location_new):
	direction = ''
	if location[1] - location_new[1] > 0:
		direction += 'S'
	elif location[1] - location_new[1] < 0:
		direction += 'N'
	if location[0] - location_new[0] > 0:
		direction += 'W'
	elif location[0] - location_new[0] < 0:
		direction += 'E'
	if direction is '':
		direction = 'Self'
	return direction

def getRandomTerrain():
	r = random.uniform(0, 1)
	if r > SIM_WALL_CHANCE:
		return WALL
	return GROUND

#simulate an observation from among 2^8 possible observations. 
def simulateSuccessorTerrain(location, terrainMap):
	surrounding_locations = getSurroundingPoints(location)
	surrounding_terrain = {}
	for l in surrounding_locations:
		if outOfBounds(surrounding_locations[l]):
			surrounding_terrain[l] = WALL
			continue
		if belief.expectedMap[surrounding_locations[l]] != UNKNOWN:
			r = random.uniform(0, 1)
			if r > PROB_SIM: #chance to still flip; simulate noisy sensor input
				surrounding_terrain[l] = terrainMap[surrounding_locations[l]]
				continue
		surrounding_terrain[l] = getRandomTerrain()
	return surrounding_terrain


#simulate movement from a location with a certain chance of failure
def simulateMove(location, action):
	r = random.uniform(0, 1)
	a_sim = action
	attempted_location = getNewLocation(action, location)
	successor_locations = getSurroundingPoints(location)
	if r > PROB_MOVEMENT_SUCCESS: #movement failed.
		adjacent_directions = getAdjacentDirections(action)
		a_sim = random.choice(adjacent_directions)

	location_new = getLocationInDirection(a_sim, location)
	# print location, action, location_new
	return location_new, a_sim

# def generateNewSuccessorlocations(a_sim, successor_location_terrain, location_new):
# 	new_locations = copy.copy(successor_location_terrain)
# 	surrounding_pointl = getSurroundingPoints(location_new)

# 	#assume moving into open ground for now. 
# 	if 'N' in a_sim:
# 		for l in new_locations:
# 			if 'N' in l and new_locations[l] == WALL:
# 				new_locations[l] = GROUND
# 	elif 'S' in a_sim:
# 		for l in new_locations:
# 			if 'S' in l and new_locations[l] == WALL:
# 				new_locations[l] = GROUND
# 	if 'E' in a_sim:
# 		for l in new_locations:
# 			if 'E' in l and new_locations[l] == WALL:
# 				new_locations[l] = GROUND
# 	elif 'W' in a_sim:
# 		for l in new_locations:
# 			if 'W' in l and new_locations[l] == WALL:
# 				new_locations[l] = GROUND

# 	#shuffle up the new location by randomly flipping locations. 
# 	for l in new_locations:
# 		r = random.uniform(0, 1)
# 		if r < PROB_SIM: #change of flipping location. 
# 			new_locations[l] = fliplocation(new_locations[l])
# 		elif belief.expectedMap[surrounding_points[l]] != UNKNOWN:
# 			new_locations[l] = belief.expectedMap[surrounding_points[l]]
# 	return new_locations

def simulateCollisionCheck(action_sim, successor_terrain):
	if successor_terrain[action_sim] is WALL: #if collision
		return True #collision
	return False #no collision

#observe simulated successor locations with a chance of failure
def simulateObservation(state, new_successor_terrain):
	# print new_successor_locations
	observation = {}
	for d in directions:
		r = random.uniform(0, 1)
		if getLocationInDirection(d, state.location) == state.targetLocation:
			observation[d] = TARGET
		elif r < PROB_SIM: #if observation fails
			observation[d] = fliplocation(new_successor_terrain[d])
		else:
			observation[d] = new_successor_terrain[d]
	return observation


def simulateReward(observation_sim, location_target, location_old, location_new): 
	slownessPenalty = manhattanDistance(location_target, location_old) - manhattanDistance(location_target, location_new)
	if location_new == location_target:
		# print 'reward!', location_old, location_new, location_target
		return R_TARGET_FOUND
	elif observation_sim['targetNear']:
		return R_TARGET_NEAR
	elif observation_sim['collision']:
		return R_COLLISION
	else:
		return R_MOVEMENT

#returnl observation, reward, and new location given a location and action. 
#there are 2^8 possible observations.
def blackBoxSimulator(state, action): #state contains a location, a map, and an estimated location of the reward.
	# print 'got here'
	location = state.location
	terrainMap = getMapFromCounts(state.mapCount)
	location_target = state.targetLocation
	successor_terrain = simulateSuccessorTerrain(location, terrainMap)
	location_new_sim, action_sim = simulateMove(location, action) #simulate a movement with chance of failure. 
	observe_collision = simulateCollisionCheck(action_sim, successor_terrain)
	if not observe_collision:
		new_surrounding_locations = simulateSuccessorTerrain(location_new_sim, terrainMap)
	else:
		new_surrounding_locations = successor_terrain #don't bother generating new locations if collision
		location_new_sim = location #location does not change. 
	observation_sim = simulateObservation(state, new_surrounding_locations)
	observation_sim['collision'] = observe_collision
	noise = random.randint(-TARGET_SENSOR_NOISE, TARGET_SENSOR_NOISE)
	noisyDistance = noise + manhattanDistance(location, location_target)
	observation_sim['targetNear'] = noisyDistance < TARGET_SENSE_THRESH
	#print observation_sim
	state_new = Particle(location_new_sim, location_target, state.mapCount)
	reward_sim = simulateReward(observation_sim, location_target, location, location_new_sim)
	# print state.location, action, state_new.location, reward_sim
	return state_new, observation_sim, reward_sim




# def particleFilter(belief, action, observation):

# 	#get new particles proportional to weights
# 	weights = []
# 	locations_new = []
# 	for i in range(len(belief.locations)):
# 		l = random.choice(belief.locations)
# 		s_new = moveParticle(s, action)
# 		locations_new.append(s_new)
# 		weight = observationProb(observation, s_new, action)
# 		weights.append(weight)

# 	normalizer = 1.0/float(sum(weights))
# 	weights*=normalizer
# 	return [locations, weights]


# def resample(particles, weights):
#     newparticles = []
#     index = random.randint(0, len(particles)-1)
#     beta = 0.0
#     maxWeight = max(weights)
#     for _ in range(len(particles)):5
#         beta += random.random() * 2.0 * maxWeight
#         while beta > weights[index]:
#             beta -= weights[index]
#             index = (index + 1) % len(particles)
#         newParticle = copy.copy(particles[index])
#         newParticles.append(newParticle)
#     return newParticles



#return reward based on belief. how close we are to target, whether or not we hit a wall,
#etc. perhapl only implement if I'm doing another algorithm besidel MCTS
def rba(belief, action):
	print "implement me!"

def rsa(location, action):
	if dig in action:
		return DIGCOST
	else:
		if cave[location[0]][location[1]]:
			print "implement me"

#perhaps for comparison to MCTl we can use forward search to find out what we should do next
#in the immediate area. 
def forwardSearch(belief, depth):
	if depth == 0:
		return (None, U(belief))
	(action_opt, utility_opt) = (None, float('-inf'))
	for action in actions:
		utility = rba(belief, action)
		for observation in observations:
			belief_new = belief.updateBelief(action, observation)
			(action_new, utility_new) = forwardSearch(belief_new, depth-1)
			utility += gamma*P(o, b, a)*utility_new
		if utility > utility_opt:
			(action_opt, utility_opt) = (action, utility)
	return (action_opt, utility_opt)

def pi_1(state):
	action = None
	for i in range(len(directions)):
		action = random.choice(directions)
		location_new = getLocationInDirection(action, state.location)
		if state.getTerrain(location_new) != WALL:
			break
	return action

def pi_2(state):
	action = None
	for i in range(len(directions)):
		action = min(((manhattanDistance(getLocationInDirection(d, state.location), state.targetLocation)), d) 
			for d in directions)[1]
		location_new = getLocationInDirection(action, state.location)
		if state.getTerrain(location_new) != WALL:
			break
	return action

def pi_0(state):
	# return random.choice([pi_1(state), pi_2(state)])
	return pi_1(state)



#this part is for MCTS. it calls the black-box simulator to try to guesl what the next
#observation should be given the action taken. (you get one observation per time unit.)
def rolloutPOMDP(belief, depth, policy):
	if depth == 0:
		return 0
	action = policy(belief)
	location = belief.samplelocation()
	(location_new, observation, reward) = blackboxPOMDP(location, action)
	belief_new = updateBelief(belief, action, observation)
	return reward + gamma*rolloutPOMDP(belief_new, depth-1, policy)

# unclear whether I should use a rollout based on locations or beliefs so I made both. 
# I'll probably end up going with rolloutPOMDP
def rolloutMDP(state, depth, policy):
	totalReward = 0
	currentState = state
	for i in range(1, depth + 1):
		action = policy(currentState)
		(currentState, _ , reward) = blackBoxSimulator(currentState, action)
		totalReward += (.99**i)*reward
		if currentState.location == currentState.targetLocation:
			break
	return totalReward

def getLogNh(Nh):
	if Nh == 0.0:
		return 0
	else:
		return math.log(Nh)

def getSqrtNhNha(logNh, Nha):
	if logNh == 0.0 and Nha == 0.0:
		return 0.0
	else:
		return math.sqrt(logNh/Nha)

def getObservationDictFromString(observationString):
	key_value_list = observationString.split(" ")
	observation = {}
	for key_value in key_value_list:
		key_value_duo = key_value.split(".")
		key = key_value_duo[0]
		value = int(key_value_duo[1])
		observation[key] = value
	# print observation
	return observation

def getObservationStringFromDict(observation):
	observationList = [str(k)+"." + str(int(observation[k])) for k in sorted(observation.keys())]
	return " ".join(observationList)

def QInit(history, action):
	if len(list(history))!=0:
		#if possible collision, return collision cost. 
		prevObsStr = history[-1]
		prevAct = history[-2]
		prevObs = getObservationDictFromString(prevObsStr)
		# print prevObs
		# print 'WAAAAAAAGGGGGGGGHHHHHHHHHHHH'
		if prevObs[action] == WALL:
			return R_COLLISION 
		elif prevObs[action] == TARGET:
			# print 'target found sim'
			return R_TARGET_FOUND
		elif prevObs['targetNear']:
			return R_TARGET_NEAR 
		else:
			return R_MOVEMENT
	return 0

#global dictionaries. 
T = set()
Q = {}
N = {}

#the main method. The big boy. 
def MCTS(belief, depth, policy, historyInit=tuple()):

	def simulate(state, history, depth):
		# print 'depth', depth
		if depth == 0:
			return 0
		if history not in T: #each history is a tuple of action/observations. 
			# print 'new hist', history
			N[history] = {}
			Q[history] = {}
			for a in directions:
				N[history][a] = 1                                                                                    
				Q[history][a] = QInit(history, a)
			T.add(history)
			v = rolloutMDP(state, depth, policy)
			return v
		# print Q[history]
		Nh = sum(N[history].values())
		logNh = getLogNh(Nh)
		action = max((Q[history][a] + EXPLORATION_CONSTANT*getSqrtNhNha(logNh, N[history][a]), a) for a in directions)[1]
		(state_new, observation, reward) = blackBoxSimulator(state, action)
		history_new = tuple(list(history) + [action] + [getObservationStringFromDict(observation)]) #append action and observation to hist tuple
		q = reward + gamma*simulate(state_new, history_new, depth-1)
		N[history][action] += 1
		Q[history][action] += (q - Q[history][action])/N[history][action]
		return q

	#wrapper loop
	cnt = 0
	while True:
		s = belief.sampleParticle()
		simulate(s, historyInit, depth)
		if cnt < MCTS_ITERS:
			cnt += 1
			# print cnt
		else:
			# print 'cnt', cnt
			break

	# print historyInit
	#returnl the arg-max action given the history etc. 
	return max((Q[historyInit][action], action) for action in directions)[1]


#I think this was an early case of what I should do
#maybe I can use ffb for comparasin. 
def fastInformedBound(location, observations, alpha_matrix_old):
	alpha = {}
	successor_locations = getSuccessorLocations(location)
	for action in actions:
		alpha_a = rsa(location, action) + gamma*sum(max(sum(observationProb(o, s, a)*transitionProb(location, s, a)*alpha_old[a][l]
																	for l in successor_locations)\
																	for a in actions)\
																	for o in observations)
		alpha[action]


#adds additional location uncertainty. Stochacisity in the location space to test the strength of POMDP algorithm. 
def collapseCaveRandom(cave):
	collapsePoint = (random.randint(0, CAVEDIM-1), random.randint(0, CAVEDIM-1))
	for i in range(-COLLAPSE_CONSTANT, COLLAPSE_CONSTANT+1):
		if collapsePoint[0]+i >= 0 and collapsePoint[0]+i < CAVEDIM:
			for j in range(-COLLAPSE_CONSTANT, COLLAPSE_CONSTANT+1):
				if collapsePoint[1]+j >= 0 and collapsePoint[1]+j < CAVEDIM:
					if random.random()>.35:
						cave[collapsePoint[0]+i][collapsePoint[1]+j] = WALL

def updateCave():
	collapseCaveRandom(cave)

# def testMap():
# 	cave[ROBOT_START_LOCATION[0], ROBOT_START_LOCATION[1]] = ROBOT
# 	tlocationB = (11, 5)
# 	tActions = getSuccessorLocations(ROBOT_START_LOCATION)
# 	total = 0

# 	for a in tActions:
# 		# total +=
# 		print transitionProb(ROBOT_START_LOCATION, tlocationB, a)

def randomWalk():
	d = random.choice(directions)
	oldlocation = copy.copy(agent.location)
	agent.move(d)
	# updateCave(oldlocation)

def createNewHistoryRoot(actionReal, observationsReal):
	return tuple([actionReal, getObservationStringFromDict(observationsReal)])

def mctsWalk(historyRoot):
	print 'tree root', historyRoot
	actionReal = MCTS(belief, MCTS_DEPTH, pi_0, historyRoot)
	oldlocation = copy.copy(agent.location)
	agent.move(actionReal)
	agent.observe()
	print 'chosen action:', actionReal
	print 'actual observ.:', agent.observations
	print 'actual location:', agent.location
	belief.updateBelief(actionReal, agent.observations)
	print 'estimated location:', belief.expectedPosition
	return createNewHistoryRoot(actionReal, agent.observations)



#should I do this randomly or should it be fixed? probably random. 
# def placeTargetInCave():


def main():
	#################init stuff####################
	

	

	# estimatedCave = belief.getEstimatedMapPlot()
	# # plt.figure(1)
	# img2 = plt.imshow(estimatedCave,interpolation='nearest',
	# 					cmap = cmap,norm=norm)
	# plt.ion()
	# plt.show()

	# # make a color bar
	# plt.colorbar(img,cmap=cmap,
	#                 norm=norm,boundaries=bounds,ticks=[-5,0,5,])

	b = Belief()
	updateCaveLocation(ROBOT_START_LOCATION, ROBOT)
	updateCaveLocation(TARGET_LOCATION, TARGET)
	agent.setlocation(ROBOT_START_LOCATION)

	# tell imshow about color map so that only set colorl are used
	img1 = plt.imshow(list(reversed(zip(*cave))),interpolation='nearest',
						cmap = cmap,norm=norm)
	plt.ion()
	plt.show()

	################init stuff over###############
	cnt = 0
	previousActionObservation = tuple()
	while(True):

		# estimatedCave = belief.getEstimatedMapPlot()
		# # plt.figure(1)
		# img2.set_data(estimatedCave)
		# plt.draw()

		plt.pause(.1)

		previousActionObservation = mctsWalk(previousActionObservation)
		# print previousActionObservation


		img1.set_data(list(reversed(zip(*cave))))
		plt.draw()
		plt.pause(.1)

		if agent.location == TARGET_LOCATION: #if agent finds target, end loop. 
			print '*************** target found ****************'
			break

		cnt += 1
		print cnt
		# updateCave()

if __name__ == "__main__":
	main()