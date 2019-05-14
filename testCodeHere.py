import matplotlib.pyplot as plt
from scipy.stats import truncnorm

CAVEDIM = 100
EDGE_THICKNESS = int(CAVEDIM/50)
lower, upper = 0+EDGE_THICKNESS, CAVEDIM-EDGE_THICKNESS
mu, sigma = CAVEDIM/2 + CAVEDIM/10, CAVEDIM/5 #mean is slightly skewed away from robot start location
N = 2

def generateTargetLocation():
	while True:
		X = truncnorm((lower-mu)/sigma,(upper-mu)/sigma,loc=mu,scale=sigma) #this doesn't seem to work very well...
		samples = X.rvs(N)
		location = [int(round(s)) for s in samples]
		(x, y) = location
		if x > lower and y > lower and x < upper and y < upper: #this is necessary because truncnorm behaves oddly
			return tuple(location)


while True:
	(x, y) = generateTargetLocation()

	if x < lower or y < lower or x > upper or y > upper:
		print 'out of bounds.'
		X = truncnorm((lower-mu)/sigma,(upper-mu)/sigma,loc=mu,scale=sigma)
		fig, ax = plt.subplots(2, sharex=True)
		ax[0].hist(X.rvs(10000), normed=True)
		plt.show()
		break