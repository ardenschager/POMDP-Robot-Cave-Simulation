import matplotlib


CAVEDIM = 50
B1 = {6,7,8}
S1 = {3,4,5,6,7,8}

B2 = {5, 6, 7, 8}
S2 = {5, 6, 7, 8}

BOX_SIZE = 6
EDGE_THICKNESS = 2

UNKNOWN_BOUND = -8
WALL_BOUND = -6
MIDDLE_BOUND = 0
GROUND_BOUND = 6

UNKNOWN = -7
WALL = -1
GROUND = 1
ROBOT = 7
TARGET = 9

TARGET_SENSE_THRESH = 10
TARGET_SENSOR_NOISE = 1
TARGET_NOT_DETECTED = 1000

BALANCE_CONSTANT = 4.2
COLLAPSE_CONSTANT = 3

MCTS_DEPTH = 500
MCTS_ITERS = 100
SIM_WALL_CHANCE = .5
gamma = .99

ROLLOUT_DEPTH = 500

NUMBER_PARTICLES = 80

TARGET_NOT_DETECTED = 999

#for sim
R_COLLISION = -100
R_MOVEMENT = -2
R_TARGET_NEAR = -1
R_TARGET_FOUND = 1000

ROBOT_START_LOCATION = (10, 5)
TARGET_LOCATION = (20, 20)


PROB_MOVEMENT_SUCCESS = .9
PROB_OBSERVATION_SUCCESS = .95

PROB_SIM = .2

EXPLORATION_CONSTANT = 500 #experiment with this.
MAP_COUNT_INIT = 10

cmap = matplotlib.colors.ListedColormap(['gray','black', 'white','red', 'green'])
bounds=[UNKNOWN_BOUND, WALL_BOUND,MIDDLE_BOUND,GROUND_BOUND,8,10] #if 7, robot; if 9, target
norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
cave_file = 'cave2.txt'