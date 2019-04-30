import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import numpy as np
from PIL import Image
from queue import PriorityQueue
import math
import itertools
import random
from scipy.spatial.transform import Rotation as R

THETASTAR = True # can turn off or on
bikelength = 20 # Specify the bike length
stepsize = 50 # specify arclength for steps

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
	tol = 5
	sol = None
	G[start] = [] # add vertex
	cameFrom = {} # for extracting trajectory
	cameFrom[start] = None


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
		if qnew != qnear:
			cameFrom[qnew] = qnear
		if (L2norm(qnew,goal) < tol): #within tolerance
			print('found goal!!!!')
			sol = qnew
			break
		# bias towards goal

	return sol,G,cameFrom

def rand_conf(mean):
	randx,randy = np.random.normal(mean, [0.5*imarray.shape[0],0.5*imarray.shape[1]], 2)
	clipped = np.array([randx,randy])
	np.clip([randx,randy], [0,0], [imarray.shape[0]-1,imarray.shape[1]-1], out=clipped)
	#randx = random.randint(1,imarray.shape[0]-1)
	#randy = random.randint(1,imarray.shape[1]-1)
	#print(clipped)
	return (int(clipped[0]),int(clipped[1]))

def draw_bicycle(bike_loc,theta,alpha):
	# draw the bicycle
	
	# Draw the bike frame and direction theta
	bikeframe = [bikelength,0,0]
	r = R.from_euler('z', theta, degrees=True)
	bikeframe = r.apply(bikeframe)
	x = bike_loc[0]
	y = bike_loc[1]
	plt.plot([x,bikeframe[0]+x],[y,bikeframe[1]+y], color='blue',linewidth=5) # plot body
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

def draw_path(bike_loc,theta,alpha,arclength):
	# Draw a path taken by the bike

	draw_bicycle(bike_loc,theta,alpha)
	x = bike_loc[0]
	y = bike_loc[1]
	# Find pivot point of bike and vector of frame of bike
	bikeframe = [bikelength,0,0]
	r = R.from_euler('z', theta, degrees=True)
	bikeframe = r.apply(bikeframe)
	pivot = (bikeframe[0]+x,bikeframe[1]+y)
	
	# Find vector normal to the bike front wheel
	bikewheel = [bikelength/2,0,0]
	w = R.from_euler('z',alpha+90,degrees=True)
	bikewheel = w.apply(bikewheel)
	bikewheel = r.apply(bikewheel)

	# Get vector normal to frame of bike
	r = R.from_euler('z', 90, degrees=True)
	other_bikeframe = bikeframe
	bikeframe = r.apply(bikeframe)

	v1 = bikeframe[:2]
	v2 = bikewheel[:2]
	bikeorigin = np.array(bike_loc)
	
	try:	
		#curved driving if the matrix is not nearly singular or singular
		a1,b1,c1=linefrompoints(bikeorigin,np.add(v1,bikeorigin))
		a2,b2,c2=linefrompoints(pivot,np.add(v2,pivot))

		a = np.array([[a1,b1],[a2,b2]])
		b = np.array([c1,c2])

		# solve for ICC
		intersection = np.linalg.solve(a,b)

		if np.linalg.cond(a) > 1000000:
			#matrix is near singular
			raise Exception("Nearly singular")

		plt.scatter(intersection[0],intersection[1],s=10,color='red',zorder=50)
		plt.plot([bikeorigin[0],intersection[0]],[bikeorigin[1],intersection[1]],color='orangered')

		rad = np.linalg.norm(np.subtract(bikeorigin,intersection))

		r = R.from_euler('z',arclength_to_angle(rad,arclength),degrees=True)
		if alpha<0:
			r = R.from_euler('z',-arclength_to_angle(rad,arclength),degrees=True)
		v = np.subtract(bikeorigin,intersection)
		v = [v[0],v[1],0]
		v = r.apply(v)

		newbikeorigin = intersection[0]+v[0],intersection[1]+v[1]
		plt.plot([intersection[0],newbikeorigin[0]],[intersection[1],newbikeorigin[1]],color='lime')
		
		greenline = np.subtract(newbikeorigin,intersection)
		greenline = greenline/np.linalg.norm(greenline)

		orangeline = np.subtract(bikeorigin,intersection)
		orangeline = orangeline/np.linalg.norm(orangeline)
		
		dot = np.dot(np.array([1,0]),greenline)
		det = greenline[0]*0-1*greenline[1]

		dot2 = np.dot(np.array([1,0]),orangeline)
		det2 = orangeline[0]*0-1*orangeline[1]

		dot3 = np.dot(greenline,orangeline)
		det3 = orangeline[0]*greenline[1]-greenline[0]*orangeline[1]

		angle = np.rad2deg(np.arctan2(det,dot))
		angle2 = np.rad2deg(np.arctan2(det2,dot2))
		angle3 = np.rad2deg(np.arctan2(det3,dot3))

		angle = angle
		angle2= angle2
		print(angle,angle2)

		if alpha<0:
			arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor='magenta',linestyle='--')
		else:
			arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor='magenta',linestyle='--')

		
		ax.add_patch(arc)

		if alpha<0:
			draw_bicycle(newbikeorigin,-90-angle,alpha)
		else:
			draw_bicycle(newbikeorigin,90-angle,alpha)

	except Exception as e:
		print(e)
		# Straight line driving in direction of v1 or v2
		vec = other_bikeframe/np.linalg.norm(other_bikeframe)
		newbikeorigin = np.add(vec[:2]*arclength,bikeorigin)
		plt.plot([bikeorigin[0],newbikeorigin[0]],[bikeorigin[1],newbikeorigin[1]],color='magenta',linestyle='--')
		draw_bicycle(newbikeorigin[0],newbikeorigin[1],theta,alpha)

def steer(bikeorigin, theta, bikegoal, thetagoal):

	plt.plot([bikeorigin[0],bikegoal[0]],[bikeorigin[1],bikegoal[1]])
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
	
	plt.plot([midpoint[0],midpoint[0]+bisector[0]],[midpoint[1],midpoint[1]+bisector[1]],color='red')
	plt.plot([bikeorigin[0],bikeorigin[0]+bikeframe[0]],[bikeorigin[1], bikeorigin[1]+bikeframe[1]])

	try:	
		#curved driving if the matrix is not nearly singular or singular
		a1,b1,c1=linefrompoints(midpoint,np.add(midpoint,bisector[:2]))
		a2,b2,c2=linefrompoints(bikeorigin,np.add(bikeorigin,bikeframe[:2]))

		a = np.array([[a1,b1],[a2,b2]])
		b = np.array([c1,c2])

		# solve for ICC
		intersection = np.linalg.solve(a,b)
		plt.scatter(intersection[0],intersection[1])
		if np.linalg.cond(a) > 1000000:
			#matrix is near singular
			raise Exception("Nearly singular")
	
		plt.scatter(intersection[0],intersection[1],s=10,color='red',zorder=50)
		plt.plot([bikeorigin[0],intersection[0]],[bikeorigin[1],intersection[1]],color='orangered')

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
		r = R.from_euler('z', -90, degrees=True)
		steervector = r.apply(steervector)
		plt.plot([pivot[0],intersection[0]],[pivot[1],intersection[1]])
		plt.plot([pivot[0],pivot[0]+steervector[0]],[pivot[1],pivot[1]+steervector[1]])

		steervector=steervector[:2]
		bikeframe = bikeframe[:2]
		dot = np.dot(bikeframe,steervector)
		det = steervector[0]*bikeframe[1]-bikeframe[0]*steervector[1]
		angle = -np.rad2deg(np.arctan2(det,dot))

		if angle<-90:
			angle = angle+180
		if angle >90:
			angle = angle-180
		print(angle)
		#if alpha<0:
		#	arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor='magenta',linestyle='--')
		#else:
		#	arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor='magenta',linestyle='--')

		#ax.add_patch(arc)

		circ = patches.Circle(intersection, radius=rad,fill=False,edgecolor='magenta',linestyle='--')
		ax.add_patch(circ)

		draw_bicycle(bikeorigin,theta,angle)
		draw_bicycle(bikegoal,thetagoal,0)
	except Exception as e:
		print(e)
	#plt.plot([intersection[0],bisector[0]],[intersection[1],bisector[1]])
	

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

"""
solution,graph,camefrom = rrt((2,2),(287,60))

if solution:
	plt.scatter(solution[0],solution[1],color='red')

for key, value in graph.items():
	for item in value:
		plt.plot([key[0],item[0]], [key[1],item[1]], linewidth=1)

try:
	a = solution
	b = camefrom[a]
	while True:
		if b is None:
			break
		plt.plot([a[0],b[0]], [a[1],b[1]], color='red',linewidth=1)
		a = b # child
		b = camefrom[b] # parent
except Exception as e:
	print(e)
	print(a)
"""

#draw_bicycle(25,25,45,45)
#draw_bicycle(50,50,90,25)
#draw_path((100,150),170,30,stepsize)
steer((0,0),80,(50,52),0)
steer((50,52),0,(80,100),90)
#steer((0,0),10,(50,52),0)

plt.show()