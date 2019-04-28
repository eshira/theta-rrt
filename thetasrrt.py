import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from PIL import Image
from queue import PriorityQueue
import math
import itertools
import random

THETASTAR = True # can turn off or on

def heuristic(node, goal):
	# Wrapper for convenience, easy to swap out what to use here
	return L2norm(node, goal)

def L2norm(node0, node1):
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

def rrt(start,goal):
	K=1000 # Number of vertices in the tree
	deltaq = 10 # incremental distance
	G = {} # graph
	tol = 10
	sol = None
	G[start] = [] # add vertex

	for k in range(1,K):
		while (True):
			qrand = rand_conf(goal)
			# If qrand fell on the tree or in obstacle
			if (not freespace(qrand)) and (not qrand in G.keys()):
				continue
			keys = list(G.keys())
			index = np.argmin([L2norm(item,qrand) for item in keys])
			qnear = keys[index]
			vector = np.array(list(np.subtract(qrand,qnear))) # points from qnear to qrand
			if(np.linalg.norm(vector)>0.000001):
				vector = deltaq*vector/np.linalg.norm(vector)
			if np.linalg.norm(vector) > np.linalg.norm(np.array(list(np.subtract(qrand,qnear)))):
				# just go to the qrand, otherwise overshoot
				qnew = qrand
			else:
				qnew = tuple(np.add(qnear,vector).astype(int)) # test; later would move qnew incremental deltaq in direction qrand
			if (not valid(qnew)):
				continue
			if not lineofsight(qnew,qnear):
				continue
			else:
				break
		if not (qnew in G.keys()):
			G[qnew] = [] # vertex
		G[qnear].append(qnew) # add edge
		if (L2norm(qnew,goal) < tol): #within tolerance
			print('found goal!!!!')
			sol = qnew
			break
		# bias towards goal

	return sol,G

def rand_conf(mean):
	randx,randy = np.random.normal(mean, [0.5*imarray.shape[0],0.5*imarray.shape[1]], 2)
	clipped = np.array([randx,randy])
	np.clip([randx,randy], [0,0], [imarray.shape[0]-1,imarray.shape[1]-1], out=clipped)
	#randx = random.randint(1,imarray.shape[0]-1)
	#randy = random.randint(1,imarray.shape[1]-1)
	#print(clipped)
	return (int(clipped[0]),int(clipped[1]))


img = Image.open('map2.png').convert('1')
imarray = np.array(img)

imgplot = plt.imshow(img)

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

solution,graph = rrt((2,2),(287,60))
if solution:
	plt.scatter(solution[0],solution[1],color='red')

for key, value in graph.items():
	for item in value:
		plt.plot([key[0],item[0]], [key[1],item[1]], linewidth=1)

plt.show()