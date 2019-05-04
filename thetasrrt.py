import matplotlib.pyplot as plt
import matplotlib.image as mpimg # for drawing images
import matplotlib.patches as patches # for drawing arcs and circles
import numpy as np
from PIL import Image # for doing some stuff with images
from queue import PriorityQueue # for A*/Theta*
import math
import itertools # for some neighbor expansion stuff
import random
from scipy.spatial.transform import Rotation as R # for consistently doing rotations

THETASTAR = True # can turn off or on
stepsize = 10 # specify arclength for steps
tol = 5 # tolerance for goal xy
tolang = 25 # tolerance for final angle to goal angle
xydev = 0.025 # multiplier (times image dimensions) for standard deviation for normal dist for xy goal biasing
# Relative weighting between xy and angle
weightxy = .8 # The weight for the angle will be the complement

bikelength = 5 # Specify the bike frame length
LEFTCONSTRAINT = -45
RIGHTCONSTRAINT = 45
FORWARDONLY = True

def heuristic(node, goal):
	# Wrapper for convenience, easy to swap out what to use here
	return L2norm(node, goal)

def L2norm(node0, node1):
	# Dunno why I didn't just use numpy
	return math.sqrt(pow((node1[0]-node0[0]),2)+pow((node1[1]-node0[1]),2))

def valid(node):
	# Node not out of bounds
	if ( (node[0]<0) or (node[1]<0) ):
		return False
	elif ( (node[0]>=imarray.shape[0]) or (node[1]>=imarray.shape[1]) ):
		return False
	else:
		return True

def freespace(node):
	# True if freespace, False if obstacle
	if imarray[node[1]][node[0]]:
		return True
	else:
		return False

def getprunedneighbors(node,parent):
	# Get only pruned neighbors of a node
	pruned = []

	# Compute and adjust magnitude of direction
	direction = np.subtract(node,parent)
	if (direction[0] !=0): np.sign(direction[0])
	if (direction[1] !=0): np.sign(direction[1])

	# Add the natural neighbor (in the direction of the step)
	pruned.append(np.add(node,direction))
	# If horizontal move, check above and below
	if direction[1]==0:
		for delta in [-1,1]:
			check = np.add(node,(0,delta))
			if not freespace(check):
				pruned.append(np.add(check,direction))
	# If vertical move, check left and right
	if direction[0]==0:
		for delta in [-1,1]:
			check = np.add(node,(delta,0))
			if not freespace(check):
				pruned.append(np.add(check,direction))

	# For a diagonal move, also add the other two natural neighbors
	if (direction[0]!=0 and direction[1]!=0):
		pruned.append((node[0]+direction[0],node[1]))
		pruned.append((node[0],node[1]+direction[1]))
	# Check flanking, add the one past the flaking
	check = np.add(node,(direction[0],0))
	if not freespace(check):
		pruned.append(np.add(check,(direction[0],0)))
	checl = np.add(node,(0,direction[1]))
	if not freespace(check):
		pruned.append(np.add(check,(0,direction[1])))
	return [item for item in pruned if (valid(item) and freespace(item))]

def getneighbors(node):
	listofneighbors = []
	# Get the neighbors of the node
	for delta in itertools.product([-1,0,1],repeat=2):
		if (delta!=(0,0)):
			candidate = np.subtract(node,delta)
			if(valid(candidate)):
				if(freespace(candidate)):
					#valid free neighbor
					listofneighbors.append(tuple(candidate))
	return listofneighbors

def reconstruct(node,pathmap):
	path = []
	while(True):
		try:
			path.append(node)
			node = pathmap[node];
		except:
			break
	return path[::-1]

def lineofsight(node1,node2):
	for px in bresenham(node1,node2):
		if not freespace(px):
			return False
	return True

def bresenham(node1,node2):
	#Bresenham's https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	x0,y0 = node1
	x1,y1 = node2
	if abs(y1 - y0) < abs(x1 - x0):
		if x0 > x1:
			return plotLineLow(x1, y1, x0, y0)
		else:
			return plotLineLow(x0, y0, x1, y1)
	else:
		if y0 > y1:
			return plotLineHigh(x1, y1, x0, y0)
		else:
			return plotLineHigh(x0, y0, x1, y1) 

def plotLineLow(x0,y0,x1,y1):
	pixels = []
	deltax = x1 - x0
	deltay = y1 - y0
	stepy = 1
	if deltay < 0:
		stepy = -1
		deltay = -deltay
	D = 2*deltay - deltax
	y = y0
	if (deltax < 0): print("?")
	for x in range(x0,x1+1):
		pixels.append((x,y))
		if D>0:
			y = y + stepy
			D = D - 2*deltax
		D = D + 2*deltay
	return pixels

def plotLineHigh(x0,y0,x1,y1):
	pixels = []
	deltax = x1 - x0
	deltay = y1 - y0
	stepx = 1
	if deltax < 0:
		stepx = -1
		deltax = -deltax
	D = 2*deltax - deltay
	x = x0
	if (deltay < 0): print("????")
	for y in range(y0,y1+1):
		pixels.append((x,y))
		if D>0:
			x = x + stepx
			D = D - 2*deltay
		D = D + 2*deltax
	return pixels

def astar(start,goal): # Pass in two tuples in the form (x,y)
	if not valid(start) or not valid(goal):
		print("Start or goal is not valid. Error.")
		return False
	if not freespace(start) or not freespace(goal):
		print("Start or goal is inside an obstacle. Error.")
		return False

	# Set up data structures
	closedSet = set([]) # The set of nodes already evaluated
	openPQ = PriorityQueue() # Candidates for evaluation
	openSet = set([])
	cameFrom = {} # Keys are nodes, values are the predecessor node
	gScore = {} # cost from start to node structure

	# Initialize with start node
	#openPQ.put((heuristic(start,goal),start)) #put the start into the open pq
	#openSet.add(start) #and into the open set
	#gScore[start]=0 # start to start costs 0

	gScore[start]=0
	initialnodes = getneighbors(start)
	for n in initialnodes:
		gScore[n] = L2norm(start,n)
		cameFrom[n]=start
		openSet.add(n)
		openPQ.put((gScore[n]+heuristic(n,goal),n))
	closedSet.add(start)

	# loop until found goal or openPQ is emtpy
	while not openPQ.empty():
		# Get the lowest priority node, remove from openset and add to closed
		current = openPQ.get()[1]
		if current in closedSet:
			# This is here because we leave old copies instead of removing from openPQ
			# Instead of properly updating priorities
			continue

		#Lazy theta*
		if THETASTAR:
			if not lineofsight(current, cameFrom[current]):
				myneighbors = [item for item in getneighbors(current) if item in closedSet]
				values = [gScore[item]+L2norm(item,current) for item in myneighbors]
				cameFrom[current] = myneighbors[np.argmin(values)]
				gScore[current] = values[np.argmin(values)]

		openSet.remove(current)
		closedSet.add(current)

		# If you found the goal, nearly done
		if (current==goal):
			print("Expanded nodes:",len(closedSet))
			return reconstruct(current,cameFrom)

		# Get the neighbors, and for each:
		neighbors = getneighbors(current)
		for neighbor in neighbors:
			# If it isn't already in the closedSet (if it is, ignore it):
			if neighbor not in closedSet:

				g = gScore[current]+L2norm(neighbor,current)
				if neighbor not in openSet:
					gScore[neighbor] = g
					openPQ.put((g+heuristic(neighbor,goal),neighbor))
					openSet.add(neighbor)
					cameFrom[neighbor]=current

				else: # neighbor is already in open set
					if g < gScore[neighbor]: # We're doing better with this path; update entry in openPQ
						gScore[neighbor] = g
						cameFrom[neighbor] = current
						# remove old copy won't be necessary because
						# the lower priority copy will get taken off first and added to open set
						openPQ.put((g+heuristic(neighbor,goal),neighbor))

				if (THETASTAR):
					# Theta star, any angle search modification
					if True: #lineofsight(cameFrom[current],neighbor): # delay line of sight check, lazy theta*
						# If the gScore for neighbor thru parent of current is better than gscore of neighbor otherwise
						g = gScore[cameFrom[current]]+L2norm(cameFrom[current],neighbor)
						if g < gScore[neighbor]:
							cameFrom[neighbor] = cameFrom[current]
							gScore[neighbor] = g
							# remove old copy won't be necessary because
							# the lower priority copy will get taken off first and added to open set
							openPQ.put((g+heuristic(neighbor,goal),neighbor))

	return False

def standardangle(angle):
	while angle >180:
		angle = angle-360
	while angle <= -180:
		angle = angle+360
	return angle

def rrt(start,goal):
	start = (start[0],standardangle(start[1]))
	goal = (goal[0],standardangle(goal[1]))
	K=500 # Number of vertices in the tree
	deltaq = 10 # incremental distance
	G = {} # graph
	sol = None
	G[start] = [] # add vertex
	cameFrom = {} # for extracting trajectory
	cameFrom[start] = None

	for k in range(1,K):
		qrand = (rand_conf(goal))
		qrand = (qrand[0],standardangle(qrand[1]))
		
		# If qrand falls on an obstacle or is in the set of keys
		if (not freespace(qrand[0])):
			print('notfreespace')
			continue
		if (qrand in G.keys()):
			print("in keys")
			continue
		keys = list(G.keys())
		index = np.argmin([L2norm(item[0],qrand[0]) for item in keys])
		qnear = keys[index]
		
		#finalang,steerang = steer(qnear[0], qnear[1], qrand[0], qrand[1],plot=True)
		#(bikegoal,final_angle),(steerangle,arclength, interection, rad)
		#(bikegoal,theta),(0,arclength, None,rad) for straight line
		qdrive,u = steer(qnear[0], qnear[1], qrand[0], qrand[1],plot=False)
		qnew = ((int(qdrive[0][0]),int(qdrive[0][1])),qdrive[1])

		finalang = qdrive[1]
		steerang = u[0]

		finalang = standardangle(finalang)
		steerang = standardangle(steerang)
		if (steerang<LEFTCONSTRAINT) or (steerang>RIGHTCONSTRAINT): #constraints on steering
			continue
		if (not valid(qnew[0])):
			continue
		if not lineofsight(qnew[0],qnear[0]):
			continue
		if not (qnew in G.keys()):
			G[qnew] = [] # vertex

		# To do, make draw_path deal with straight linees
		#draw_path(qnear,qdrive,u)

		G[qnear].append(qnew) # add edge

		if qnew != qnear:
			cameFrom[qnew] = qnear

		r1 = R.from_euler('z', qnew[1], degrees=True)
		r2 = R.from_euler('z', goal[1], degrees=True)
		diff = r1.inv()*r2
		diff = diff.as_euler('xyz')[2]
		diff = np.rad2deg(diff)

		if (L2norm(qnew[0],goal[0]) < tol) and (abs(diff)<tolang): #within tolerance
			print('found goal!!!!')
			sol = qnew
			break
		# bias towards goal

	return sol,G,cameFrom

def rand_conf(mean):
	#randx = random.randint(1,imarray.shape[0]-1)
	#randy = random.randint(1,imarray.shape[1]-1)
	randx,randy = np.random.normal(mean[0], [xydev*imarray.shape[0],xydev*imarray.shape[1]], 2)
	clipped = np.array([randx,randy])
	np.clip([randx,randy], [0,0], [imarray.shape[0]-1,imarray.shape[1]-1], out=clipped)
	
	randtheta = standardangle(np.rad2deg(np.random.uniform(0,2*np.pi)))
	#stddevdeg = 5
	#randtheta = np.random.normal(standardangle(mean[1]),stddevdeg,1)[0]
	#randtheta = standardangle(randtheta)
	return ((int(clipped[0]),int(clipped[1])),randtheta)
	#return (int(clipped[0]),int(clipped[1])),np.rad2deg(randtheta)

def draw_bicycle(bike_loc,theta,alpha,color='blue'):
	# Draw the bike frame and direction theta
	bikeframe = [bikelength,0,0]
	r = R.from_euler('z', theta, degrees=True)
	bikeframe = r.apply(bikeframe)
	x = bike_loc[0]
	y = bike_loc[1]
	plt.plot([x,bikeframe[0]+x],[y,bikeframe[1]+y], color=color,linewidth=5) # plot body
	plt.quiver(x,y,bikeframe[0]/2,bikeframe[1]/2,facecolor='red',edgecolor='black',linewidth=0.5,headwidth=2.5,zorder=10,angles='xy', scale_units='xy', scale=1)

	# Draw the front wheel direction
	bikewheel = [bikelength/2,0,0]
	w = R.from_euler('z',alpha,degrees=True)
	bikewheel = w.apply(bikewheel)
	bikewheel = r.apply(bikewheel)
	pivot = (bikeframe[0]+x,bikeframe[1]+y)
	plt.quiver(pivot[0],pivot[1],bikewheel[0],bikewheel[1],facecolor='yellow',edgecolor='black',linewidth=0.5,headwidth=2.5,zorder=10,angles='xy', scale_units='xy', scale=1)
	
	# Draw an arrow 90 degrees to the front wheel
	bikewheel = [bikelength/2,0,0]
	w = R.from_euler('z',alpha+90,degrees=True)
	bikewheel = w.apply(bikewheel)
	bikewheel = r.apply(bikewheel)
	pivot = (bikeframe[0]+x,bikeframe[1]+y)
	plt.quiver(pivot[0],pivot[1],bikewheel[0],bikewheel[1],facecolor='cyan',edgecolor='black',linewidth=0.5,headwidth=2.5,zorder=10,angles='xy', scale_units='xy', scale=1)

def draw_path(bike1,bike2,u):
	# u = (0,arclength, intersection,rad)
	intersection = u[2]

	if intersection is not None:
		rad = u[3]

		icc_to_bike2 = np.subtract(bike2[0],intersection)
		icc_to_bike2 = icc_to_bike2 /np.linalg.norm(icc_to_bike2)

		icc_to_bike1 = np.subtract(bike1[0],intersection)
		icc_to_bike1 = icc_to_bike1/np.linalg.norm(icc_to_bike1)
		
		dot = np.dot(np.array([1,0]),icc_to_bike2)
		det = icc_to_bike2[0]*0-1*icc_to_bike2[1]

		dot2 = np.dot(np.array([1,0]),icc_to_bike1)
		det2 = icc_to_bike1[0]*0-1*icc_to_bike1[1]

		dot3 = np.dot(icc_to_bike2,icc_to_bike1)
		det3 = icc_to_bike1[0]*icc_to_bike2[1]-icc_to_bike2[0]*icc_to_bike1[1]

		angle = np.rad2deg(np.arctan2(det,dot))
		angle2 = np.rad2deg(np.arctan2(det2,dot2))
		angle3 = np.rad2deg(np.arctan2(det3,dot3))

		# Draw the bike frame and direction theta
		bikeframe = [bikelength,0,0]
		r = R.from_euler('z', bike1[1], degrees=True)
		bikeframe = r.apply(bikeframe)
		pivot = (bikeframe[0]+bike1[0][0],bikeframe[1]+bike1[0][1])
		steervector = np.subtract(pivot,intersection)
		steervector = [steervector[0],steervector[1],0]
		r = R.from_euler('z', 90, degrees=True)
		steervector = r.apply(steervector)
		steervector=steervector[:2]
		bikeframe = bikeframe[:2]
		dot = np.dot(bikeframe,steervector)
		det = steervector[0]*bikeframe[1]-bikeframe[0]*steervector[1]
		steerangle = -np.rad2deg(np.arctan2(det,dot))
		steerangle = standardangle(steerangle)
		flip=False
		if FORWARDONLY and ((steerangle>90) or (steerangle<-90)):
			steerangle = steerangle+180
			flip = True

		color='lime'
		#print(u[0])
		if  u[0]<-0: #right left coloring
			# if you want to make it color based on backward/forward, use ((u[0]>90) or (u[0]<-90)):
			color='cyan'
		
		if u[0]<0 or flip:
			arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor=color,linestyle='--')
		else:
			arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor=color,linestyle='--')

		ax.add_patch(arc)

	
	#draw_bicycle(bike1[0],bike1[1],u[0],color='red')
	draw_bicycle(bike2[0],bike2[1],0,color='blue')

def steer(bikeorigin, theta, bikegoal, thetagoal,plot=False):
	# Find the midpoint and bisector of the line between origin and goal
	midpoint = np.array([0.5*(bikeorigin[0]+bikegoal[0]),0.5*(bikeorigin[1]+bikegoal[1])])
	bisector = np.subtract(bikeorigin,bikegoal)
	bisector = [bisector[0],bisector[1],0]

	# Find pivot point of bike and vector of frame of bike
	bikeframe = [bikelength,0,0]
	r = R.from_euler('z', theta, degrees=True)
	bikeframe = r.apply(bikeframe)
	
	# Get vector normal to frame of bike
	r = R.from_euler('z', 90, degrees=True)
	bikeframe = r.apply(bikeframe)
	bisector = r.apply(bisector)
	
	try:	
		#curved driving if the matrix is not nearly singular or singular
		a1,b1,c1=linefrompoints(midpoint,np.add(midpoint,bisector[:2]))
		a2,b2,c2=linefrompoints(bikeorigin,np.add(bikeorigin,bikeframe[:2]))

		a = np.array([[a1,b1],[a2,b2]])
		b = np.array([c1,c2])

		# solve for ICC
		intersection = np.linalg.solve(a,b)
		if np.linalg.cond(a) > 1000000:
			#matrix is near singular
			raise Exception("Nearly singular")

		# turn radius
		rad = np.linalg.norm(np.subtract(bikeorigin,intersection))

		# Draw the bike frame and direction theta
		bikeframe = [bikelength,0,0]
		r = R.from_euler('z', theta, degrees=True)
		bikeframe = r.apply(bikeframe)
		x = bikeorigin[0]
		y = bikeorigin[1]
		pivot = (bikeframe[0]+x,bikeframe[1]+y)

		steervector = np.subtract(pivot,intersection)
		steervector = [steervector[0],steervector[1],0]
		r = R.from_euler('z', 90, degrees=True)
		steervector = r.apply(steervector)

		steervector=steervector[:2]
		bikeframe = bikeframe[:2]
		dot = np.dot(bikeframe,steervector)
		det = steervector[0]*bikeframe[1]-bikeframe[0]*steervector[1]
		steerangle = -np.rad2deg(np.arctan2(det,dot))
		
		# positive steer is right
		# if steering right, point to ICC, turn left 90
		# if steering left, point to ICC, turn rigt
		final_angle = np.subtract(bikegoal,intersection)
		final_angle = [final_angle[0],final_angle[1],0]
		
		steerangle = standardangle(steerangle)

		flip=False

		# If the bicycle is trying to steer backwards, we don't let it; ought to try the path forward
		# TO DO: find out which is the longer path and choose that if FORWARDONLY is false
		if FORWARDONLY and ((steerangle>90) or (steerangle<-90)):
			steerangle = steerangle+180
			steerangle = standardangle(steerangle)
			flip = True
	
		c_ccw = 0
		# Front Right and Back left steering case
		if ((steerangle >= 0) and (steerangle <90)) or ((steerangle <= -90) and (steerangle > -180)):
			r = R.from_euler('z', 90, degrees=True)
			c_ccw = -90
		# Front left and back right
		else:
			r = R.from_euler('z',-90,degrees=True)
			c_ccw = 90

		final_angle = r.apply(final_angle)
		final_angle=final_angle[:2]
		
		testvec = [1,0]
		dot = np.dot(testvec,final_angle)
		det = final_angle[0]*testvec[1]-testvec[0]*final_angle[1]
		final_angle = -np.rad2deg(np.arctan2(det,dot))

		# All Angle Goal Point in CYAN
		vec2 = [1,0,0]
		thetagoal2 = thetagoal+c_ccw
		r2 = R.from_euler('z',thetagoal2,degrees=True)
		vec2 = r2.apply(vec2)
		# Normalize that vector and make it length radius
		vec2 = rad*vec2/np.linalg.norm(vec2)
		vec2 = vec2[:2]
		# The point that it lands on achieves the angle.
		goalanglept = np.add(intersection,vec2)
		#plt.scatter(goalanglept[0],goalanglept[1], s=30, color='cyan')
		
		# First mix point in RED
		mix = weightxy*standardangle(final_angle) + (1-weightxy)*standardangle(thetagoal)
		vec2 = [1,0,0]
		thetagoal2 = (mix)+c_ccw
		r2 = R.from_euler('z',thetagoal2,degrees=True)
		vec2 = r2.apply(vec2)
		# Normalize that vector and make it length radius
		vec2 = rad*vec2/np.linalg.norm(vec2)
		vec2 = vec2[:2]
		mix1 = vec2
		# The point that it lands on achieves the angle.
		goalanglept = np.add(intersection,vec2)
		#plt.scatter(goalanglept[0],goalanglept[1], s=100,color='green')
		#draw_bicycle(goalanglept,mix,0,color='yellow')
		#final angle vs thetagoal, mix.

		# Second mix point in LIME
		mix = standardangle(mix+180)
		vec2 = [1,0,0]
		thetagoal2 = (mix)+c_ccw
		r2 = R.from_euler('z',thetagoal2,degrees=True)
		vec2 = r2.apply(vec2)
		# Normalize that vector and make it length radius
		vec2 = rad*vec2/np.linalg.norm(vec2)
		vec2 = vec2[:2]
		mix2 = vec2
		# The point that it lands on achieves the angle.
		goalanglept = np.add(intersection,vec2)
		#plt.scatter(goalanglept[0],goalanglept[1], s=100,color='green')
		#draw_bicycle(goalanglept,mix,0,color='lime')

		#circle = patches.Circle(intersection, radius=rad,color='pink',linestyle='--',fill=None)
		#ax.add_patch(circle)
		#draw_bicycle(goalanglept,thetagoal,0,color='pink')

		# Vector mix1
		# Vector mix2 is mix1 rotated 180
		# Compare differences in angle to bike1

		# Vector max arclength away from start position
		# icc_to_originbike

		icc_to_originbike = np.subtract(bikeorigin,intersection)
		icc_to_originbike = icc_to_originbike/np.linalg.norm(icc_to_originbike)

		dot = np.dot(mix1,icc_to_originbike)
		det = icc_to_originbike[0]*mix1[1]-mix1[0]*icc_to_originbike[1]

		dot1 = np.dot(mix2,icc_to_originbike)
		det1 = icc_to_originbike[0]*mix2[1]-mix2[0]*icc_to_originbike[1]
		
		angleA = np.rad2deg(np.arctan2(det,dot))
		angleB = np.rad2deg(np.arctan2(det1,dot1))

		if ((angleA>angleB) and steerangle<0) or  ((angleA<=angleB) and steerangle>=0):
			mix1 = mix2
			mix = weightxy*standardangle(final_angle) + (1-weightxy)*standardangle(thetagoal)-180
			mix = standardangle(mix)
		else:
			mix = weightxy*standardangle(final_angle) + (1-weightxy)*standardangle(thetagoal)
			mix = standardangle(mix)
		# The point that it lands on achieves the angle.
		goalanglept = np.add(intersection,mix1)
		#draw_bicycle(goalanglept,mix,steerangle,color='pink')

		icc_to_goalbike = np.subtract(goalanglept,intersection)
		icc_to_goalbike = icc_to_goalbike /np.linalg.norm(icc_to_goalbike )
		
		dot = np.dot(np.array([1,0]),icc_to_goalbike)
		det = icc_to_goalbike[0]*0-1*icc_to_goalbike[1]

		dot2 = np.dot(np.array([1,0]),icc_to_originbike)
		det2 = icc_to_originbike[0]*0-1*icc_to_originbike[1]

		dot3 = np.dot(icc_to_goalbike,icc_to_originbike)
		det3 = icc_to_originbike[0]*icc_to_goalbike[1]-icc_to_goalbike[0]*icc_to_originbike[1]

		angle = np.rad2deg(np.arctan2(det,dot))
		angle2 = np.rad2deg(np.arctan2(det2,dot2))
		angle3 = np.rad2deg(np.arctan2(det3,dot3))


		color='magenta'
		if flip:
			color='blue'
		if plot:
			if flip or steerangle<0:
				arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor=color,linestyle='--')
			else:
				arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor=color,linestyle='--')

			ax.add_patch(arc)
			#draw_bicycle(bikeorigin,theta,steerangle,color='blue')
			draw_bicycle(bikegoal,final_angle,0,color='blue')			

		arclength = 0 # dummy placeholder for now
		# Return the final bike state and useful info relating to u, the controls
		return (goalanglept,mix),(steerangle,arclength,intersection,rad)

	except Exception as e: # Singular matrix; straight line driving
		print(e)
		arclength = 0 # dummy placeholder for now
		return (bikegoal,theta),(0,arclength, None,None)
		#return theta,0

def arclength_to_angle(radius, arclength):
	return arclength*360/(np.pi*2*radius)

def fixangle(angle):
	while angle < 0:
		angle = angle+ 360
	while angle > 360:
		angle = angle -360
	return angle

def linefrompoints(p,q):
	a = q[1]-p[1]
	b = p[0]-q[0]
	c = a*p[0]+b*p[1]
	return a,b,c

img = Image.open('blank.png').convert('1')
imarray = np.array(img)

imgplot = plt.imshow(img)
plt.grid(True)
ax = plt.gca()
#mainpath = astar((280,0),(8,280))
mainpath = None

if mainpath:
	path=[]

	for first, second in zip(mainpath,mainpath[1:]):
		pathsegment = bresenham(first,second)
		path = path+pathsegment

	if (path):
		print("Found path")
		xs = [item[0] for item in path]
		ys = [item[1] for item in path]
		c_ = range(len(path))
		plt.scatter(xs,ys, s=10,c=c_,cmap="winter")
else:
	pass
	#print("Didn't find path")

begin = ((20,20),standardangle(135))
end = ((105,105),standardangle(45))
solution,graph,camefrom = rrt( begin , end )

draw_bicycle(begin[0],begin[1],0,color='orangered')
draw_bicycle(end[0],end[1],0,color='green')

try:
	a = solution
	b = camefrom[a]
	while True:
		if b is None:
			break
		steer(b[0],b[1],a[0],a[1],plot=True)
		a = b # child
		b = camefrom[b] # parent
except Exception as e:
	print(e)
	print(a)

plt.show()