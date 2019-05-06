""" Search algorithms and helper functions
	Astar, lazy theta *
"""
from main import *
from queue import PriorityQueue # for A*/Theta*
import itertools # for some neighbor expansion stuff
import math

def heuristic(node, goal):
	# Wrapper for convenience, easy to swap out what to use here
	return L2norm(node, goal)

def L2norm(node0, node1):
	# Dunno why I didn't just use numpy
	return math.sqrt(pow((node1[0]-node0[0]),2)+pow((node1[1]-node0[1]),2))

def valid(node):
	# Node not out of bounds
	if ( (int(node[0])<0) or (int(node[1])<0) ):
		return False
	elif ( (int(node[0])>=builtins.imarray.shape[0]) or (int(node[1])>=builtins.imarray.shape[1]) ):
		return False
	else:
		return True

def freespace(node):
	# True if freespace, False if obstacle
	if imarray[int(node[1])][int(node[0])]:
		return True
	else:
		return False

def lineofsight(node1,node2):
	node1 = (int(node1[0]),int(node1[1]))
	node2 = (int(node2[0]),int(node2[1]))	
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

def getCirclePoints(xc,yc,p,q):
	pixels=[]
	for value1  in [-p,p,-q,q]:
		for value2 in [-p,p,-q,q]:
			if abs(value1)==abs(value2):
				continue
			pixel = (xc+value1,yc+value2)
			if valid(pixel):
				pixels.append(pixel)
	return pixels

def getCircle(center,r,draw=False):
	xc = int(center[0])
	yc = int(center[1])
	r = int(r)
	x = r
	y = 0
	d = 1 - r
	pixels = []
	while x>y:
		pixels = pixels+getCirclePoints(xc, yc, x, y)
		y = y + 1
		if d < 0:
			d = d + 2*y + 1
		else:
			x = x - 1
			d = d + 2*(y-x) + 1
	
	test = (xc+int(round(r*0.5*math.sqrt(2))),yc+int(round(r*0.5*math.sqrt(2))))
	testlist = [(test[0]+1,test[1]),(test[0]-1,test[1]),(test[0],test[1]+1),(test[0],test[1]-1)]
	drawmore=True
	for item in testlist:
		if item in pixels:
			drawmore=False
	if drawmore:
		additional = []
		additional.append((xc+round(r*0.5*math.sqrt(2)),yc+round(r*0.5*math.sqrt(2))))
		additional.append((xc-round(r*0.5*math.sqrt(2)),yc-round(r*0.5*math.sqrt(2))))
		additional.append((xc+round(r*0.5*math.sqrt(2)),yc-round(r*0.5*math.sqrt(2))))
		additional.append((xc-round(r*0.5*math.sqrt(2)),yc+round(r*0.5*math.sqrt(2))))
		for item in additional:
			if valid(item):
				pixels.append(item)
	if draw:
		for item in pixels:
			builtins.imarray[item[1],item[0]]=0
	return pixels

def getArc(begin,land,u,draw=False):
	if u[1] is None: # you tried to draw an arc for a straight line
		# To do, return the straight line with the line drawing algo
		return False
	else:
		# Get the full circle reprsented by pixels
		pixels = getCircle(u[1],u[2])
		# Compute the angles to the beginning and end of the arc
		beginangle = rrt.anglebetween([1,0],np.subtract(begin[0],u[1]))
		endangle = rrt.anglebetween([1,0],np.subtract(land[0],u[1]))
		if u[0]<0: # left turn case (CCW)
			diff = rrt.anglediff(beginangle,endangle)
		else: # right turn case (CW)
			diff = rrt.anglediff(endangle,beginangle)
			if diff<0:
				diff = 360+diff

		remaining = []
		# For every circle pixel, figure out if you need to keep it
		for item in pixels:
			# Angle to this pixel
			pxangle = rrt.anglebetween([1,0],np.subtract(item,u[1]))
			if u[0]>0:
				forwardofbegin = rrt.anglediff(pxangle,beginangle)
				backwardofgoal = rrt.anglediff(endangle,pxangle)
			else:
				forwardofbegin = rrt.anglediff(beginangle,pxangle)
				backwardofgoal = rrt.anglediff(pxangle,endangle)
			if diff<180:
				if (forwardofbegin >= 0) and (backwardofgoal>=0):
					include=True
				else:
					include=False
			else:
				if (forwardofbegin >= 0) or (backwardofgoal>=0):
					include=True
				else:
					include=False
			if include:
				remaining.append(item)
				if draw: builtins.imarray[item[1],item[0]]=0

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

def drawpath(mainpath):
	# plot the path returned by A* or Theta*
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

	# Set up the problem by expanding the start node
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
			""" Because we leave old copies instead of removing from openPQ
				(instead of properly updating priorities)
				we need to skip (continue) if we find current in closedSet
			"""
			continue

		#Lazy theta*, check line of sight when expanding and fix parent if problem arises
		if builtins.THETASTAR:
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
							
	# If you made it this far, you never found a goal						
	return False