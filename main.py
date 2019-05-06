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
builtins.K=500 # Number of vertices in the tree
builtins.showtree = False
builtins.maxdrivedist = 10 # Max steering arclength/straight line distance to drive
builtins.tol_xy = 5 # tolerance for goal xy
builtins.tol_ang = 15 # tolerance for final angle to goal angle
builtins.weightxy = .6 # Mix between target xy and angle in steer(). Angle weight is the complement
builtins.xystdv = 0.03 # Stddev fac (multiply by image dimensions) for normal dist of xy position in rand_conf()
builtins.anglestdv = 100 # Stddev for normal dist of angle in rand_conf()

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
	begin = ((100,50),rrt.standardangle(-179))
	end = ((20,20),rrt.standardangle(0))
	#begin = ((20,20),rrt.standardangle(0))
	#end = ((100,50),rrt.standardangle(-179))

	solution,graph,camefrom = rrt.rrt( begin , end )
	if solution is not None:
		rrt.drawpath(solution,camefrom)
	else:
		print("Didn't find solution")
		rrt.drawtree(begin,graph,camefrom)
		
		# find nearest on graph to solution? pick that???
		nearest,mindist = rrt.findnearest(graph,end)
		rrt.draw_bicycle(nearest[0],nearest[1],0,color='blue')
					
	rrt.draw_bicycle(begin[0],begin[1],0,color='red')
	rrt.draw_bicycle(end[0],end[1],0,color='lime')
	
	#begin = ((103,45),rrt.standardangle(-80))
	#end = ((103,48),rrt.standardangle(-90))
	#rrt.steer(begin[0],begin[1],end[0],end[1],plot=True)

	# Finally show the plot
	plt.show()