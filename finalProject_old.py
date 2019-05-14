import matplotlib
import matplotlib.pyplot as plt
import pickle
import math
import random
import numpy as np
import copy
import time

CAVEDIM = 50
B1 = {6,7,8}
S1 = {3,4,5,6,7,8}

B2 = {5, 6, 7, 8}
S2 = {5, 6, 7, 8}

WALLBOUND = -6
MIDDLEBOUND = 0
GROUNDBOUND = 6
WALL = -1
GROUND = 1
ROBOT = 7
GOAL = 9
BALANCECONSTANT = 4.6
COLLAPSECONSTANT = 3
MCTS_ITERS = 5

NUMBER_PARTICLES = 100

PROB_SUCCESS = .8

directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
actions = directions + [d+'Dig' for d in directions]
directions.append('Self')

#load the cave model
f = open('cave.txt', 'rb')
cave = pickle.load(f)

#the agent model
class Robot:
	def __init__(self):
		x = 0
		y = 0
		#the actual location, which the robot is unaware of. 
		state = (x, y)
		o = {}
		#up, right, down, left

	#actual move with random chance of failing
	def robotMove(d):
		weights = []
		successor_states = getSuccessorStates(state)
		for s in successor_states:
			weights.append(transitionProb(state, s, action))
		state = weightedPick(successor_states, weights)

	#actual observation with random chance of failing
	def updateObservations(cave):

		def observe(distant_state):
			d = distance(distant_state, self.state)
			p = PROB_SUCCESS**d
			if cave[x][y] == GOAL:
				return GOAL
			if random.uniform(0,1)<p:
				return cave[x][y]
			else:
				return cave[x][y]*-1

		observations = {}
		for d in directions:
			s = getNewState(d, state)
			observations[d] = observe(s)
			for dp in directions:
				if dp not in d:
					break
				sp = getNewState(dp, s)
				observations[d+d] = observe(sp)
		self.o = observations

class Belief:
	def __init__(self):
		weights = []
		states = []

	def sampleState():
		return weightedPick(states, weights)


			


#returns euclidian distance between two points
def distance(state_a, state_b):
	return int(round(math.sqrt((state_a[0] - state_b[0])**2 + (state_a[1] - state_b[1])**2)))

def getSuccessorStates(state):
	successor_states = {}
	for d in directions:
		successor_states[d] = getNewState(d, state)
	return successor_states

def getNewState(d, state):
	state_new = (0, 0)
	if d is 'Self':
		return state
	if 'N' in d:
		state_new[1] = state[1] + 1
	elif 'S' in d:
		state_new[1] = state[1] - 1
	if 'E' in d:
		state_new[0] = state[0] + 1
	elif 'W' in d:
		state_new[0] = state[0] - 1
	return state_new

def observationProb(observation, state_new, action):
	print "implement me!"



def transitionProb(state, state_new, action):
	successor_states = getSuccessorStates(state)
	if action not in successor_states.values():
		return 0
	elif getNewState(action, state) == state_new:
		return PROB_SUCCESS
	else:
		return (1-PROB_SUCCESS)/(len(directions)-1)

def weightedPick(states, weights):
	r = random.uniform(0, sum(weights))
	s = 0.0
	for i in len(states):
		s += weights[i]
		if r<s: return states[i]
	return states[-1]


def resample(particles, weights):
	    newParticles = []
	    index = random.randint(0, len(particles)-1)
	    beta = 0.0
	    maxWeight = max(weights)
	    for _ in range(len(particles)):
	        beta += random.random() * 2.0 * maxWeight
	        while beta > weights[index]:
	            beta -= weights[index]
	            index = (index + 1) % len(particles)
	        newParticle = copy.copy(particles[index])
	        newParticles.append(newParticle)
	    return newParticles


def moveParticle(state, action):
	weights = []
	successor_states = getSuccessorStates(state)

	for s in successor_states:
		weights.append(transitionProb(state, s, action))

	return weightedPick(successor_states, weights)


#update belief using a particle filter

def rba(belief, action):
	print "implement me!"

def rsa(state, action):
	if dig in action:
		return DIGCOST
	else:
		if cave[state[0]][state[1]]:
			print "implement me"
	

def forwardSearch(belief, depth):
	if depth is 0:
		return (None, utility(belief))
	(action_opt, utility_opt) = (None, float('-inf'))
	for action in actions:
		utility = reward(belief, action)
		for observation in observations:
			belief_new = updateBelief(belief, action, observation)
			(action_new, utility_new) = forwardSearch(belief_new, depth-1)
			utility += gamma*P(o, b, a)*utility_new
		if utility > utility_opt:
			(action_opt, utility_opt) = (action, utility)
	return (action_opt, utility_opt)

# def pi_0(belief):


def rollout(belief, depth, policy):
	if depth is 0:
		return 0
	action = policy(belief)
	s = belief.sampleState()
	(s_new, o, r) = blackbox(s, a)
	belief_new = updateBelief(belief, action, o)
	return r + gamma*rollout(belief_new, depth-1, policy)

Tree = set()
Q = dict(dict())
N = dict(dict())

def MCts(belief, depth, policy):

	def simulate(state, history, depth):
		if depth is 0:
			return 0
		if history not in tree:
			for a in actions:
				N[history][actions] = 0.0
				q[history][actions] = 0.0
			tree.add(tuple(history))
			return rollout(state, depth, policy)
		N = sum(N.values())
		a = max((Q[tuple(history)][a] + c*math.sqrt(math.log(N)/N[history][a]), a) for a in actions)[1]
		(s_new, o, r) = blackbox(state, a)
		q = r + gamma*simulate(s_new, history+str(a)+str(o), depth-1)
		N[history][a] += 1
		Q[history][a] += (q-Q[history][a])/N[history][a]


	history = []
	for _ in range(MCTS_ITERS):
		s = belief.sampleState()
		simulate(s, h, d)

	return max((Q[tuple(history)][action], a) for a in actions)[1]



def fastInformedBound(state, observations, alpha_matrix_old):
	alpha = {}
	successor_states = getSuccessorStates(state)
	for action in actions:
		alpha_a = rsa(state, action) + gamma*sum(max(sum(observationProb(o, s, a)*transitionProb(state, s, a)*alpha_old[a][s]
																	for s in successor_states)\
																	for a in actions)\
																	for o in observations)
		alpha[action]



def collapseCaveRandom(cave):
	collapsePoint = (random.randint(0, CAVEDIM-1), random.randint(0, CAVEDIM-1))
	for i in range(-COLLAPSECONSTANT, COLLAPSECONSTANT+1):
		if collapsePoint[0]+i >= 0 and collapsePoint[0]+i < CAVEDIM:
			for j in range(-COLLAPSECONSTANT, COLLAPSECONSTANT+1):
				if collapsePoint[1]+j >= 0 and collapsePoint[1]+j < CAVEDIM:
					if random.random()>.35:
						cave[collapsePoint[0]+i][collapsePoint[1]+j] = WALL


def updateCave(cave):
	collapseCaveRandom(cave)

# make a color map of fixed colors
cmap = matplotlib.colors.ListedColormap(['black','white','red', 'yellow'])
bounds=[WALLBOUND,MIDDLEBOUND,GROUNDBOUND,8,10] #if 7, robot; if 9, victim
norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)

# tell imshow about color map so that only set colors are used
img = plt.imshow(cave,interpolation='nearest',
                    cmap = cmap,norm=norm)
plt.ion()
plt.show()

# # make a color bar
# plt.colorbar(img,cmap=cmap,
#                 norm=norm,boundaries=bounds,ticks=[-5,0,5,])



while(True):
	# updateCave(cave)
	img = plt.imshow(cave,interpolation='nearest',
                    cmap = cmap,norm=norm)
	plt.draw()
	time.sleep(10)
	break
