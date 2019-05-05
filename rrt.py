""" RRT, steer, and helper functions
"""
from main import *
from scipy.spatial.transform import Rotation as R # for consistently doing rotations
import search
import matplotlib.patches as patches # for drawing arcs and circles

def standardangle(angle):
	while angle >180:
		angle = angle-360
	while angle <= -180:
		angle = angle+360
	return angle

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

def linefrompoints(p,q):
	a = q[1]-p[1]
	b = p[0]-q[0]
	c = a*p[0]+b*p[1]
	return a,b,c

def angle_to_arclength(radius, angle):
	while angle<0:
		angle = angle+360
	return (np.pi*2*radius)*angle/360

def rand_conf(mean):
	""" Generate a random xy loc based on a normal distribution
		biased to the goal xy and clipped to image dimensions
		Std dev specified by user parameters at top of file
	"""
	randx,randy = np.random.normal(mean[0], [xystdv*imarray.shape[0],xystdv*imarray.shape[1]], 2)
	clipped = np.array([randx,randy])
	np.clip([randx,randy], [0,0], [imarray.shape[0]-1,imarray.shape[1]-1], out=clipped)
	
	""" Generate a random angle based on a normal distribution
		biased to goal angle
		Std dev specified by user parameters at top of file
	"""
	randtheta = np.random.normal(standardangle(mean[1]),anglestdv,1)[0]
	randtheta = standardangle(randtheta)
	return ((int(clipped[0]),int(clipped[1])),randtheta)

def arclength_to_angle(radius, arclength):
	return arclength*360/(np.pi*2*radius)

def drawpath(solution, camefrom):
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

def rrt(start,goal):
	# Set up start, goal, tree, and parent map
	start = (start[0],standardangle(start[1]))
	goal = (goal[0],standardangle(goal[1]))
	G = {} # graph
	sol = None
	G[start] = [] # add vertex
	cameFrom = {} # for extracting trajectory
	cameFrom[start] = None

	# Create the RRT
	for k in range(1,K):
		# Get a randomly sampled point
		qrand = (rand_conf(goal))
		qrand = (qrand[0],standardangle(qrand[1]))

		# If qrand falls on an obstacle or is in the set of keys
		if (not search.freespace(qrand[0])):
			print('Not freespace')
			continue
		if (qrand in G.keys()):
			print("In keys")
			continue

		# Find qnear, the nearest point (by L2norm) in the tree to qrand
		keys = list(G.keys())
		index = np.argmin([search.L2norm(item[0],qrand[0]) for item in keys])
		qnear = keys[index]
		
		# Try to drive there
		qdrive,u = steer(qnear[0], qnear[1], qrand[0], qrand[1],plot=builtins.showtree)
		qnew = ((qdrive[0][0],qdrive[0][1]),qdrive[1])

		""" Skip if not valid for any reason """
		# Reason: Cannot achieve this steering
		if (standardangle(u[0])<LEFTCONSTRAINT) or (standardangle(u[0])>RIGHTCONSTRAINT):
			continue
		# Reason: Not in the map
		#if (not valid(qrdive[0]):
		#	print('invalid')
		#	continue
		# Reason: Path runs into obstacles
		#if not lineofsight(qnew[0],qnear[0]):
		#	print('notlineofsight')
		#	continue

		# If qnew fell on an existing node in the tree, must not overwrite it
		if not (qnew in G.keys()):
			G[qnew] = [] # vertex

		# Uncomment if you want to draw the tree
		#draw_path(qnear,qdrive,u)

		G[qnear].append(qnew) # add edge

		if qnew != qnear:
			cameFrom[qnew] = qnear

		# Compute angle difference between goal and qnew
		r1 = R.from_euler('z', qnew[1], degrees=True)
		r2 = R.from_euler('z', goal[1], degrees=True)
		diff = r1.inv()*r2
		diff = diff.as_euler('xyz')[2]
		diff = np.rad2deg(diff)

		# Within tolerance of goal
		if (search.L2norm(qnew[0],goal[0]) < tol_xy) and (abs(diff)<tol_ang):
			print('Found goal!')
			sol = qnew
			break

	# Debug, print the size of the tree, to know how many of the K iterations were thrown out
	print("Nodes in tree: ",len(G.keys()))

	return sol,G,cameFrom

def steer(bikeorigin, theta, bikegoal, thetagoal,plot=False):
	""" Steer the bike towards the bikegoal in a single step attempt
		
		Step 1: Identify a single step path
		The ICC is on the line that passes thru the bikeorigin and is orthogonal to the bike
		Choices that result in tight steering may be prohibited by constraints
		(but the RRT is responsible for skipping those)
		Put the ICC where the ICC line and the perpendicular bisector to (origin,goal) intersect
		If they are parallel then this is a straight line step.

		Step 2: Mix between hitting the goal xy perfectly and hitting the target angle
		It is not likely possible to achieve both of these in a single step
		So the goal will be a mix between these two objectives (mix weights specified by user)
		If the path to this objective exceeds the max drive distance, drive to the max distance instead 
	"""

	# Find the midpoint and bisector of the line between origin and goal
	midpoint = np.array([0.5*(bikeorigin[0]+bikegoal[0]),0.5*(bikeorigin[1]+bikegoal[1])])
	bisector = np.subtract(bikeorigin,bikegoal)
	bisector = [bisector[0],bisector[1],0]

	# Find the vector of the bike frame
	bikeframe = [bikelength,0,0]
	r = R.from_euler('z', theta, degrees=True)
	bikeframe = r.apply(bikeframe)
	
	# Rotate both vectors yielding the perp bisector and the normal of the bike
	r = R.from_euler('z', 90, degrees=True)
	bikeframe = r.apply(bikeframe)
	bisector = r.apply(bisector)
	
	# Attempt to solve for the ICC
	# (singular matrix exception will mean straight line driving)
	try:	
		# Convert to matrix form
		a1,b1,c1=linefrompoints(midpoint,np.add(midpoint,bisector[:2]))
		a2,b2,c2=linefrompoints(bikeorigin,np.add(bikeorigin,bikeframe[:2]))
		a = np.array([[a1,b1],[a2,b2]])
		b = np.array([c1,c2])

		# Solve for the intersection point of bike normal and perp bisector
		intersection = np.linalg.solve(a,b)
		if np.linalg.cond(a) > 1000000:
			# Matrix is near singular
			raise np.linalg.LinAlgError('Singular matrix')

		# Compute the turn radius
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
		if builtins.FORWARDONLY and ((steerangle>90) or (steerangle<-90)):
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
		
		# First mix point in RED
		mix = builtins.weightxy*standardangle(final_angle) + (1-builtins.weightxy)*standardangle(thetagoal)
		vec2 = [1,0,0]
		thetagoal2 = (mix)+c_ccw
		r2 = R.from_euler('z',thetagoal2,degrees=True)
		vec2 = r2.apply(vec2)
		# Normalize that vector and make it length radius
		vec2 = rad*vec2/np.linalg.norm(vec2)
		vec2 = vec2[:2]
		mix1 = vec2
		# The point that it lands on achieves the angle.
		#plt.scatter(mix1[0]+intersection[0],mix1[1]+intersection[1],color='red')

		icc_to_originbike = np.subtract(bikeorigin,intersection)
		icc_to_originbike = icc_to_originbike/np.linalg.norm(icc_to_originbike)
		
		mix = builtins.weightxy*standardangle(final_angle) + (1-builtins.weightxy)*standardangle(thetagoal)
		mix = standardangle(mix)
		
		# The point that it lands on achieves the angle.
		goalanglept = np.add(intersection,mix1)

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

		# Compare arclength of origin to goal to max arclength param
		traveldist = angle_to_arclength(rad,abs(angle3))
		if angle3<0:
			traveldist = angle_to_arclength(rad,360-abs(angle3))		
		if traveldist > builtins.maxdrivedist:
			icc_to_originbike=icc_to_originbike*rad
			maxdriveangle = arclength_to_angle(rad,builtins.maxdrivedist)
			if steerangle<0:
				maxrot = R.from_euler('z',-maxdriveangle,degrees=True)
				# get the vector to the start bike. based on steer, rotate appropriately.
				landing = maxrot.apply([icc_to_originbike[0],icc_to_originbike[1],0])
				landing=np.add(landing[:2],intersection)
				#if plot: plt.scatter(landing[0],landing[1],color='red')
				#plt.scatter(intersection[0],intersection[1],color='lime')
			else:
				maxrot = R.from_euler('z',maxdriveangle,degrees=True)
				# get the vector to the start bike. based on steer, rotate appropriately.
				landing = maxrot.apply([icc_to_originbike[0],icc_to_originbike[1],0])
				landing=np.add(landing[:2],intersection)
				#if plot: plt.scatter(landing[0],landing[1],color='blue')

			goalanglept = landing

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

			final_angle = np.subtract(goalanglept,intersection)
			final_angle = [final_angle[0],final_angle[1],0]
			final_angle = r.apply(final_angle)
			final_angle=final_angle[:2]
		
			testvec = [1,0]
			dot = np.dot(testvec,final_angle)
			det = final_angle[0]*testvec[1]-testvec[0]*final_angle[1]
			final_angle = -np.rad2deg(np.arctan2(det,dot))
			mix = final_angle

		color='magenta'
		if flip:
			color='blue'
		if plot and not ((steerangle<LEFTCONSTRAINT) or (steerangle>RIGHTCONSTRAINT)): 
			if flip or steerangle<0:
				arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor=color,linestyle='--')
			else:
				arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor=color,linestyle='--')

			ax.add_patch(arc)
			#draw_bicycle(bikeorigin,theta,steerangle,color='cyan')
			draw_bicycle(goalanglept,mix,0,color='blue')			

		# Return the final bike state and useful info relating to u, the controls
		return (goalanglept,mix),(steerangle,intersection,rad)

	except np.linalg.LinAlgError as e: # Singular matrix; straight line driving
		vector = np.subtract(bikegoal,bikeorigin) # point from bikeorigin to bikegoal
		
		if(np.linalg.norm(vector)>0.000001): #if you can normalize it
			stepvector = builtins.maxdrivedist*vector/np.linalg.norm(vector)
			if np.linalg.norm(stepvector) > np.linalg.norm(vector):
			# just go to the the goal, otherwise overshoot
				pass
			else:
				# go as far as allowed
				bikegoal = np.add(bikeorigin,stepvector)
		
		if plot:
			plt.plot([bikeorigin[0],bikegoal[0]],[bikeorigin[1],bikegoal[1]],linestyle='--',color='pink')
			draw_bicycle(bikegoal,thetagoal,0,color='blue')
		return (bikegoal,theta),(0, None,None)
		#return theta,0