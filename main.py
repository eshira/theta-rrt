import sys
from PIL import Image # for doing some stuff with images
import numpy as np
import matplotlib.pyplot as plt
import builtins
import search
import rrt
import math

"""
PARAMETERS
"""

# Search parameters
builtins.THETASTAR = True # True gives Theta* and False gives A* 

# Bike parameters
builtins.bikelength = 5 # Specify the bike frame length
builtins.FORWARDONLY = True
builtins.LEFTCONSTRAINT = -55
builtins.RIGHTCONSTRAINT = 55

# RRT parameters
builtins.K=500 # Number of vertices in the tree
builtins.showtree = False
builtins.maxdrivedist = 10 # Max steering arclength/straight line distance to drive
builtins.tol_xy = 5 # tolerance for goal xy
builtins.tol_ang = 15 # tolerance for final angle to goal angle
builtins.weightxy = .6 # Mix between target xy and angle in steer(). Angle weight is the complement
builtins.xystdv = 0.04 # Stddev fac (multiply by image dimensions) for normal dist of xy position in rand_conf()
builtins.anglestdv = 100 # Stddev for normal dist of angle in rand_conf()

# Main
if __name__ == "__main__":
	# Set up image and plotting stuff
	if len(sys.argv)>1:
		img = Image.open(sys.argv[1]).convert('1')
	else:
		img = Image.open('map2.png').convert('1')
	
	builtins.imarray = np.array(img)
	plt.grid(True)
	builtins.ax = plt.gca()

	# Do the interesting stuff here
	
	"""
	# Run and show results of path planning
	mainpath = search.astar((280,0),(8,280))
	search.drawpath(mainpath)
	"""
	
	
	# Run an RRT
	begin = ((5,20),rrt.standardangle(90))
	end = ((40,115),rrt.standardangle(0))
	#begin = ((20,20),rrt.standardangle(0))
	#end = ((100,50),rrt.standardangle(-179))

	solution,graph,camefrom = rrt.rrt( begin , end , debug=True)
	if solution is not None:
		rrt.drawpath(solution,camefrom)
	else:
		print("Didn't find solution")
		# The tree is only useful to visualize if small (limit K)
		#rrt.drawtree(begin,graph,camefrom)
		
		# find nearest on graph to solution? pick that???
		nearest,mindist = rrt.findnearest(graph,end)
		rrt.drawpath(nearest,camefrom)
		rrt.draw_bicycle(nearest[0],nearest[1],0,color='darkgray')
					
	rrt.draw_bicycle(begin[0],begin[1],0,color='red')
	rrt.draw_bicycle(end[0],end[1],0,color='lime')
	

	
	"""
	begin = ((100,45),rrt.standardangle(45))
	end = ((130,66),rrt.standardangle(90))
	#rrt.draw_bicycle(begin[0],begin[1],0,color='red')
	rrt.draw_bicycle(end[0],end[1],0,color='lime')
	landing,u = rrt.steer(begin[0],begin[1],end[0],end[1],plot=True)
	
	goodpath = True
	color='green'
	pixels = search.getArc(begin,landing,u)
	if False in [search.freespace(px) for px in pixels]:
		color='red'
	for px in pixels:
		plt.scatter(px[0],px[1],color=color,s=10)
	"""

	# Finally show the plot
	imgplot = plt.imshow(builtins.imarray,cmap='gray', vmin=0, vmax=1)
	plt.show()