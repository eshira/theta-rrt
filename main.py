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
builtins.LEFTCONSTRAINT = -65
builtins.RIGHTCONSTRAINT = 65
builtins.frontclearance = 2 # front clearance multiplier by bike length

# RRT parameters
builtins.K=300 # Number of vertices in the tree
builtins.showtree = False
builtins.maxdrivedist = 30 # Max steering arclength/straight line distance to drive
builtins.tol_xy = 10 # tolerance for goal xy
builtins.tol_ang = 45 # tolerance for final angle to goal angle
builtins.weightxy = .6 # Mix between target xy and angle in steer(). Angle weight is the complement
builtins.xystdv = 0.4 # Stddev fac (multiply by image dimensions) for normal dist of xy position in rand_conf()
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
	#theta* result, saved for later
	nearest = None
	path = [(280, 0), (73, 38), (72, 39), (33, 130), (15, 190), (8, 280), None]
	for first, second,third in zip(path, path[1:],path[2:]):
		angle1 = rrt.anglebetween([1,0],np.subtract(second,first))
		if nearest is not None:
			angle1 = nearest[1]
			first = nearest[0]
		if third==None:
			angle2 = angle1
		else:
			angle2 = rrt.anglebetween([1,0],np.subtract(third,second))		
		begin = (first,rrt.standardangle(angle1))
		end = (second,rrt.standardangle(angle2))

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
	
	
	#bike = ((16,81),rrt.standardangle(45))
	#rrt.draw_bicycle(bike[0],bike[1],0,color='red')
	#rrt.front_of_bike_clear(bike,plot=True)
	
	"""
	end = ((10,110),rrt.standardangle(-90))
	begin = ((60,110),rrt.standardangle(170))
	#rrt.draw_bicycle(begin[0],begin[1],0,color='red')
	rrt.draw_bicycle(end[0],end[1],0,color='lime')
	landing,u = rrt.steer(begin[0],begin[1],end[0],end[1],plot=True)
	rrt.drive(begin,u)
	"""
	"""
	goodpath = True
	color='green'
	circle = search.getCircle(u[1],u[2])
	pixels = search.getArc(begin[0],landing[0],u)
	
	#if False in [search.freespace(px) for px in pixels]:
	#	color='red'
	for px in circle:
		plt.scatter(px[0],px[1],color='cyan',s=10)
	for px in pixels:
		plt.scatter(px[0],px[1],color=color,s=10)
	"""

	# Finally show the plot
	imgplot = plt.imshow(builtins.imarray,cmap='gray', vmin=0, vmax=1)
	plt.show()