import sys
from PIL import Image # for doing some stuff with images
import numpy as np
import matplotlib.pyplot as plt
import builtins
import search
import rrt

"""
PARAMETERS
"""

# Search parameters
builtins.THETASTAR = True # True gives Theta* and False gives A* 

# Bike parameters
builtins.bikelength = 5 # Specify the bike frame length
builtins.FORWARDONLY = True
builtins.LEFTCONSTRAINT = -45
builtins.RIGHTCONSTRAINT = 45

# RRT parameters
builtins.K=1000 # Number of vertices in the tree
builtins.showtree = False
builtins.maxdrivedist = 10 # Max steering arclength/straight line distance to drive
builtins.tol_xy = 5 # tolerance for goal xy
builtins.tol_ang = 15 # tolerance for final angle to goal angle
builtins.weightxy = .8 # Mix between target xy and angle in steer(). Angle weight is the complement
builtins.xystdv = 0.01 # Stddev for normal dist of xy position in rand_conf()
builtins.anglestdv = 10 # Stddev for normal dist of angle in rand_conf()

# Main
if __name__ == "__main__":
    # Set up image and plotting stuff
	if len(sys.argv)>1:
		img = Image.open(sys.argv[1]).convert('1')
	else:
		img = Image.open('blank.png').convert('1')
	
	builtins.imarray = np.array(img)
	imgplot = plt.imshow(img)
	plt.grid(True)
	builtins.ax = plt.gca()

	# Do the interesting stuff here
	
	"""
	# Run and show results of path planning
	mainpath = search.astar((280,0),(8,280))
	search.drawpath(mainpath)
	"""
	
	# Run an RRT
	begin = ((20,20),rrt.standardangle(0))
	end = ((60,25),rrt.standardangle(170))

	solution,graph,camefrom = rrt.rrt( begin , end )
	rrt.drawpath(solution,camefrom)
	rrt.draw_bicycle(begin[0],begin[1],0,color='red')
	rrt.draw_bicycle(end[0],end[1],0,color='lime')

	# Finally show the plot
	plt.show()