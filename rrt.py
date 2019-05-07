""" RRT, steer, and helper functions
"""
from main import *
from scipy.spatial.transform import Rotation as R # for consistently doing rotations
import search
import matplotlib.patches as patches # for drawing arcs and circles
import itertools

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

def anglebetween(vec1,vec2):
	# Compute the angle between to vectors
	dot = np.dot(vec1[:2],vec2)
	det = vec2[0]*vec1[1]-vec1[0]*vec2[1]
	return standardangle(np.rad2deg(np.arctan2(det,dot)))

def drawpath(solution, camefrom):
	try:
		a = solution
		draw_bicycle(solution[0],solution[1],0,color='green')
		b,u = camefrom[a]
		while True:
			if b is None:
				break
			draw_path_segment(b,a,u)
			#pixels = search.getArc(b[0],a[0],u)
			if front_of_bike_clear(a,plot=False):
				front_of_bike_clear(a,plot=True)
			#for item in pixels:
				#builtins.imarray[item[1]][item[0]]=0
				#plt.scatter(item[0],item[1],s=10,color='blue')
			#steer(b[0],b[1],a[0],a[1],plot=True)
			a = b # child
			b,u = camefrom[b] # parent
	except TypeError as e:
		pass	

def drawtree(begin,graph,camefrom):
	for parent in graph.keys():
		for child in graph[parent]:
			dummy,u = camefrom[child]
			draw_path_segment(parent,child,u,colors={"left":"lightgray","right":"silver","straight":"silver","bike":"darkgray"},bikes=False)
		if not graph[parent]:
			draw_bicycle(parent[0],parent[1],0,color='darkgray')

def anglediff(angle1,angle2):
	# Compute angle difference between angle1 and angle2
	r1 = R.from_euler('z', angle1, degrees=True)
	r2 = R.from_euler('z', angle2, degrees=True)
	diff = r1.inv()*r2
	diff = diff.as_euler('xyz')[2]
	diff = np.rad2deg(diff)
	return diff

def findnearest(tree,goal):
	# find nearest on graph to solution? pick that???
	nearest = (None,None)
	for parent in tree.keys():
		for child in tree[parent]:
			distance = builtins.weightxy*search.L2norm(child[0],goal[0]) + (1-builtins.weightxy)*abs(anglediff(child[1],goal[1]))
			if nearest==(None,None):				
				nearest = (child,distance)
			else:
				if distance < nearest[1]: # Distance from child to parent
					nearest = (child,distance)
	return nearest
	
def rrt(start,goal,debug=False):
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
		if debug: print(k)
		# Get a randomly sampled point
		qrand = (rand_conf(goal))
		qrand = (qrand[0],standardangle(qrand[1]))

		# If qrand falls on an obstacle or is in the set of keys
		if (not search.freespace(qrand[0])):
			if debug: print('Qrand not in freespace')
			continue
		if (qrand in G.keys()):
			if debug: print("Qrand already in keys")
			continue

		# Find qnear, the nearest point (by L2norm) in the tree to qrand
		keys = list(G.keys())
		index = np.argmin([search.L2norm(item[0],qrand[0]) for item in keys])
		qnear = keys[index]
		
		# Try to drive there
		qdrive,u = steer(qnear[0], qnear[1], qrand[0], qrand[1])
		qnew = ((qdrive[0][0],qdrive[0][1]),qdrive[1])

		""" Skip if not valid for any reason """
		# Reason: Cannot achieve this steering
		if (standardangle(u[0])<LEFTCONSTRAINT) or (standardangle(u[0])>RIGHTCONSTRAINT):
			continue
		# Reasons to try driving less far
		if (not search.valid(qnew[0])) or (not bike_clear(qnew)) or (not front_of_bike_clear(qnew)):
			u = (u[0],u[1],u[2],u[3]/3) # to do would be decide the right distance
			qnew,u = drive(qnear, u)
		# Reason: Path runs into obstacles
		pixels = search.getArc(qnear[0],qnew[0],u)
		if False in [search.freespace(px) for px in pixels]:
			if debug: print('Path to qnew intersects obstacles')
			continue

		# If qnew fell on an existing node in the tree, must not overwrite it
		if not (qnew in G.keys()):
			G[qnew] = [] # vertex

		if builtins.showtree:
			draw_path_segment(qnear,qdrive,u)

		G[qnear].append(qnew) # add edge

		if qnew != qnear:
			cameFrom[qnew] = (qnear,u)

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

def bike_clear(bike):
	bikeframe = [bikelength,0,0]
	bikeframe = R.from_euler('z', bike[1], degrees=True).apply(bikeframe)
	pivot = np.add(bikeframe[:2],bike[0])
	pivot = (int(pivot[0]),int(pivot[1]))
	return search.lineofsight(bike[0],pivot)

def front_of_bike_clear(bike,plot=False):
	bikeframe = [bikelength*builtins.frontclearance,0,0]
	bikeframe = R.from_euler('z', bike[1], degrees=True).apply(bikeframe)
	pivot = np.add(bikeframe[:2],bike[0])
	pivot=(int(pivot[0]),int(pivot[1]))
	test = search.lineofsight(bike[0],pivot)
	if test==False: return False
	return True

def draw_path_segment(bike1,bike2,u,colors={"left":"magenta","right":"dodgerblue","straight":"red","bike":"dodgerblue"},bikes=True):
	""" Draw the path segment passed in to this function
		Useful so that we can only visualize non-culled candidates in rrt()
	"""
	intersection = u[1]
	if intersection is not None:
		rad = u[2]
		icc_to_bike2 = np.subtract(bike2[0],intersection)
		icc_to_bike1 = np.subtract(bike1[0],intersection)
		
		# Compute the angle of goal bike, origin bike, and angle between the two
		angle = anglebetween([1,0],icc_to_bike2)
		angle2 = anglebetween([1,0],icc_to_bike1)
		angle3 = anglebetween(icc_to_bike2,icc_to_bike1)

		# Compute the steering angle
		bikeframe = [bikelength,0,0]
		bikeframe = R.from_euler('z', bike1[1], degrees=True).apply(bikeframe)
		pivot = np.add(bikeframe[:2],bike1[0])
		steervector = np.subtract(pivot,intersection)
		steervector = [steervector[0],steervector[1],0]
		steervector = R.from_euler('z', 90, degrees=True).apply(steervector)
		steerangle = standardangle(-anglebetween(bikeframe[:2],steervector[:2]))
		
		flip=False
		if FORWARDONLY and ((steerangle>90) or (steerangle<-90)):
			steerangle = steerangle+180
			flip = True

		color=colors["left"]
		if  u[0]<-0: #right left coloring
			# if you want to make it color based on backward/forward, use ((u[0]>90) or (u[0]<-90)):
			color=colors["right"]
		
		if u[0]<0 or flip:
			arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor=color,linestyle='--',zorder=10)
		else:
			arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor=color,linestyle='--',zorder=10)

		ax.add_patch(arc)
	
	else:
		plt.plot([bike1[0][0],bike2[0][0]],[bike1[0][1],bike2[0][1]],linestyle='--',color=colors["straight"])
	
	if bikes:
		draw_bicycle(bike1[0],bike1[1],u[0],color=colors["bike"])
		#draw_bicycle(bike2[0],bike2[1],0,color='blue')

def drive(bikeorigin, u):
	# Return the result of steering u
	# Convert arclength to angle
	angle = arclength_to_angle(u[2],u[3])
	# Find the vector of the bike frame
	bikeframe = [bikelength,0,0]
	r_theta = R.from_euler('z', bikeorigin[1], degrees=True)
	bikeframe = r_theta.apply(bikeframe)
	bikeframe1 = R.from_euler('z',180,degrees=True).apply(bikeframe)

	bikeoriginvec = [bikeorigin[0][0],bikeorigin[0][1],0]
	icc = [u[1][0],u[1][1],0]
	bikeframe = np.add(np.subtract(bikeoriginvec,icc),bikeframe)
	bikeframe1 = np.add(np.subtract(bikeoriginvec,icc),bikeframe1)

	if u[0]<0: # left turn
		# Rotate the starting point CCW by angle
		r = R.from_euler('z',-angle,degrees=True)
	else:
		r = R.from_euler('z',angle,degrees=True)

	bikeframe = r.apply(bikeframe)
	bikeframe1 = r.apply(bikeframe1)

	bikeframe = np.add(icc[:2],bikeframe[:2])
	bikeframe1 = np.add(icc[:2],bikeframe1[:2])

	finalposition = 0.5*np.add(bikeframe1,bikeframe)
	bikeframe = np.subtract(bikeframe[:2],finalposition)
	finalangle = -anglebetween([1,0],bikeframe)
	# Rotate both vectors yielding the perp bisector and the normal of the bike
	#draw_bicycle(finalposition,finalangle,0,color='yellow')
	return ((finalposition[0],finalposition[1]),finalangle),u

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
	r_theta = R.from_euler('z', theta, degrees=True)
	bikeframe = r_theta.apply(bikeframe)
	
	# Rotate both vectors yielding the perp bisector and the normal of the bike
	r_90 = R.from_euler('z', 90, degrees=True)
	bikenormal = r_90.apply(bikeframe)
	perpbisector = r_90.apply(bisector)
	
	# Attempt to solve for the ICC
	# (singular matrix exception will mean straight line driving)
	try:	
		# Convert to matrix form
		a1,b1,c1=linefrompoints(midpoint,np.add(midpoint,perpbisector[:2]))
		a2,b2,c2=linefrompoints(bikeorigin,np.add(bikeorigin,bikenormal[:2]))
		a = np.array([[a1,b1],[a2,b2]])
		b = np.array([c1,c2])

		# Solve for the intersection point of bike normal and perp bisector
		intersection = np.linalg.solve(a,b)
		if np.linalg.cond(a) > 1000000:
			# Matrix is near singular
			raise np.linalg.LinAlgError('Singular matrix')

		# Compute the turn radius
		rad = np.linalg.norm(np.subtract(bikeorigin,intersection))

		# Find the steering angle
		pivot = np.add(bikeframe[:2],bikeorigin)
		steervector = np.subtract(pivot,intersection)
		steervector = [steervector[0],steervector[1],0]
		steervector = r_90.apply(steervector)
		steervector=steervector[:2]
		steerangle = standardangle(-anglebetween(bikeframe[:2],steervector))

		flip=False
		point_to_goal_only = False

		# If FORWARDONLY, flip any backwards steering
		# TO DO: if FORWARDONLY is false choose between forward or backwards based on length
		if builtins.FORWARDONLY and ((steerangle>90) or (steerangle<-90)):
			steerangle = steerangle+180
			steerangle = standardangle(steerangle)
			flip = True

		
		# If steering is past left or right constraint, limit it and turn along that radius
		if (steerangle<builtins.LEFTCONSTRAINT) or (steerangle>builtins.RIGHTCONSTRAINT):
			if (steerangle<builtins.LEFTCONSTRAINT):
				steerangle = builtins.LEFTCONSTRAINT
				# To get normal to front wheel, take bike normal and rotate it by the steer angle
				frontnorm = R.from_euler('z',steerangle,degrees=True).apply(bikenormal)
			if (steerangle>builtins.RIGHTCONSTRAINT):
				steerangle = builtins.RIGHTCONSTRAINT
				# To get normal to front wheel, take bike normal and rotate it by the steer angle
				frontnorm = R.from_euler('z',steerangle,degrees=True).apply(bikenormal)
			a1,b1,c1=linefrompoints(pivot,np.add(pivot,frontnorm[:2]))
			a2,b2,c2=linefrompoints(bikeorigin,np.add(bikeorigin,bikenormal[:2]))
			a = np.array([[a1,b1],[a2,b2]])
			b = np.array([c1,c2])
			intersection = np.linalg.solve(a,b)
			if np.linalg.cond(a) > 1000000:
				# Matrix is near singular
				raise np.linalg.LinAlgError('Singular matrix')
			# Compute the turn radius
			rad = np.linalg.norm(np.subtract(bikeorigin,intersection))
			point_to_goal_only = True

		
		c_ccw = 0
		# Front Right and Back left steering case
		if ((steerangle >= 0) and (steerangle <90)) or ((steerangle <= -90) and (steerangle > -180)):
			c_ccw = -90
		# Front left and back right
		else:
			c_ccw = 90

		""" Find the final angle of the bike if it were to drive to the xy goal along this path
			Note that positive steering angle means steering to the right
			if steering right, point to the ICC and then turn left 90
			if steering left, point to the ICC, and then turn right
		"""
		final_vec = np.subtract(bikegoal,intersection)
		final_vec = [final_vec[0],final_vec[1],0]
		final_vec = R.from_euler('z', -c_ccw, degrees=True).apply(final_vec)
		final_vec=final_vec[:2]
		final_angle = -anglebetween([1,0],final_vec)

		# Find the objective point and angle that is a mix between origin target xy and target angle
		mix_angle = builtins.weightxy*standardangle(final_angle) + (1-builtins.weightxy)*standardangle(thetagoal)
		
		#point_to_goal_only = False
		# For cases where turning was limited by turn radius
		if point_to_goal_only:
			# Find the angle to the goal from bike origin point to bike goal point
			mix_angle = anglebetween([1,0],np.subtract(bikegoal,bikeorigin))

		thetagoal2 = (mix_angle)+c_ccw
		mix_point1 = R.from_euler('z',thetagoal2,degrees=True).apply([1,0,0])[:2]
		mix_point1 = rad*mix_point1/np.linalg.norm(mix_point1)
		mix_point1 = np.add(intersection,mix_point1)

		thetagoal3 = thetagoal2+180
		mix_point2 = R.from_euler('z',thetagoal3,degrees=True).apply([1,0,0])[:2]
		mix_point2 = rad*mix_point2/np.linalg.norm(mix_point2)
		mix_point2= np.add(intersection,mix_point2)

		diff1 = anglediff(anglebetween([1,0],np.subtract(mix_point1,intersection)),anglebetween([1,0],np.subtract(bikegoal,intersection)))
		diff2 = anglediff(anglebetween([1,0],np.subtract(mix_point2,intersection)),anglebetween([1,0],np.subtract(bikegoal,intersection)))
		final_angle = mix_angle

		if abs(diff1)<abs(diff2):
			#Pick original mix point
			goal_point = mix_point1
		else:
			#Pick rotated mix point
			goal_point = mix_point2
			final_angle = standardangle(final_angle-180)

		if point_to_goal_only:
			goal_point = mix_point1
			#if steerangle < 0:
			#	final_angle = standardangle(final_angle+180)
			final_vec = np.subtract(goal_point,intersection)
			final_vec = [final_vec[0],final_vec[1],0]
			final_vec = R.from_euler('z', -c_ccw, degrees=True).apply(final_vec)
			final_vec=final_vec[:2]
			final_angle = -anglebetween([1,0],final_vec)
			#plt.scatter(intersection[0],intersection[1],color='red')

		# Vector pointing from ICC to new target bike
		icc_to_goalbike = np.subtract(goal_point,intersection)
		# Vector pointing from ICC to origin bike
		icc_to_originbike = np.subtract(bikeorigin,intersection)

		# Compute the angle of goal bike, origin bike, and angle between the two
		angle = anglebetween([1,0],icc_to_goalbike)
		angle2 = anglebetween([1,0],icc_to_originbike)
		angle3 = anglebetween(icc_to_goalbike,icc_to_originbike)

		# Compute the length of the path to be travelled
		r1 = R.from_euler('z',angle,degrees=True)
		r2 = R.from_euler('z',angle2,degrees=True)
		diff = r1.inv()*r2
		diff = diff.as_euler('xyz')[2]
		arcangle = np.rad2deg(diff)

		if steerangle>0: # CW driving, turning right
			if arcangle<0: # CCW angle, so take complement
				arcangle = 360+arcangle
		else: # CCW driving, turning left
			if arcangle>0: #CW angle, so take complement
				arcangle = 360-arcangle
				
		traveldist = angle_to_arclength(rad,abs(arcangle))	
		# If that length is more than allowed, update target point
		if traveldist > builtins.maxdrivedist:
			traveldist = builtins.maxdrivedist
			maxdriveangle = arclength_to_angle(rad,builtins.maxdrivedist)
			if steerangle<0:
				maxrot = R.from_euler('z',-maxdriveangle,degrees=True)
			else:
				maxrot = R.from_euler('z',maxdriveangle,degrees=True)
			# get the vector to the start bike. based on steer, rotate appropriately.
			goal_point = maxrot.apply([icc_to_originbike[0],icc_to_originbike[1],0])
			goal_point = np.add(goal_point[:2],intersection)
			icc_to_goalbike = np.subtract(goal_point,intersection)
			#if plot: plt.scatter(goal_point[0],goal_point[1],color='lime')

			# Compute the angle of goal bike, origin bike, and angle between the two
			angle = anglebetween([1,0],icc_to_goalbike)
			angle2 = anglebetween([1,0],icc_to_originbike)
			angle3 = anglebetween(icc_to_goalbike,icc_to_originbike)

			final_vec = np.subtract(goal_point,intersection)
			final_vec = [final_vec[0],final_vec[1],0]
			final_vec = R.from_euler('z', -c_ccw, degrees=True).apply(final_vec)
			final_vec=final_vec[:2]
			final_angle = -anglebetween([1,0],final_vec)

		# If plot=True, draw this path
		color='magenta'
		if flip:
			color='blue'

		if plot: 
			if flip or steerangle<0:
				arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle, theta1=0, theta2=-angle3,edgecolor=color,linestyle='--',zorder=10)
			else:
				arc = patches.Arc(intersection, rad*2, rad*2, angle=-angle2, theta1=0, theta2=angle3,edgecolor=color,linestyle='--',zorder=10)

			ax.add_patch(arc)
			draw_bicycle(bikeorigin,theta,steerangle,color='cyan')
			draw_bicycle(goal_point,final_angle,0,color='blue')			

		# Return the final bike state and useful info relating to u, the controls
		return (goal_point,final_angle),(steerangle,intersection,rad,traveldist)

	except np.linalg.LinAlgError as e: # Singular matrix; straight line driving
		vector = np.subtract(bikegoal,bikeorigin) # points from bikeorigin to bikegoal
		if(np.linalg.norm(vector)>0.000001): #if you can normalize it
			stepvector = builtins.maxdrivedist*vector/np.linalg.norm(vector)
			if np.linalg.norm(stepvector) > np.linalg.norm(vector):
			# You can reach the goal, so go to that
				pass
			else:
				# Go as far as you can, falling short of goal
				bikegoal = np.add(bikeorigin,stepvector)
		
		if plot:
			plt.plot([bikeorigin[0],bikegoal[0]],[bikeorigin[1],bikegoal[1]],linestyle='--',color='pink')
			draw_bicycle(bikegoal,thetagoal,0,color='blue')

		return (bikegoal,theta),(0, None,None,traveldist)