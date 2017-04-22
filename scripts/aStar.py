import rospy, tf, numpy, math, random, Queue, collections, heapq
#from implementation import *
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

class Cell:
	def __init__(self, xLoc, yLoc, reachable):
		self.reachable = reachable
		self.loc = (xLoc, yLoc)
		self.fCost = 0
		self.gCost = 0
		self.parent = None
		self.visited = False

class SimpleGraph:
	def __init__(self):
		self.edges = {}

	def neighbors(self, id):
		return self.edges[id]

class Queue:
	def __init__(self):
		self.elements = collections.deque()

	def empty(self):
		return len(self.elements) == 0

	def put(self, x):
		self.elements.append(x)

	def get(self):
		return self.elements.popleft()

class PriorityQueue:
	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))

	def get(self):
		return heapq.heappop(self.elements)[1]

class SquareGrid:
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.walls = []

	def in_bounds(self, id):
		(x, y) = id
		return 0 <= x < self.width and 0 <= y < self.height

	def passable(self, id):
		return id not in self.walls

	def neighbors(self, id):
		(x, y) = id
		results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
		if(x + y)%2 == 0: results.reverse()
		results = filter(self.in_bounds, results)
		results = filter(self.passable, results)
		return results

class GridWithWeights(SquareGrid):
	def __init__(self, width, height):
		#super().__init__(self, width, height)
		self.width = width
		self.height = height
		self.walls = []
		self.weights = {}

	def cost(self, from_node, to_node):
		return self.weights.get(to_node, 1)

def heuristic(a, b):
	(x1, y1) = a
	(x2, y2) = b
	return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
	global grid

	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0

	while not frontier.empty():
		current = frontier.get()

		if current == goal:
			break

		for next in graph.neighbors(current):
			new_cost = cost_so_far[current] + graph.cost(current, next)
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				priority = new_cost + heuristic(goal, next)
				frontier.put(next, priority)
				came_from[next] = current

	return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
	current = goal
	path = [current]
	while current != start:
		current = came_from[current]
		path.append(current)
	path.append(start)
	path.reverse()
	return path

def getMap(msg):
	global grid
	global offSetX
	global offSetY
	global resolution
	global height
	global width

	height = msg.info.height
	width = msg.info.width
	offSetX = msg.info.origin.position.x
	offSetY = msg.info.origin.position.y
	grid = GridWithWeights(width, height)
	resolution = msg.info.resolution

	wallList = []

	#create index variable
	i = 0 #index for outer loop (keeps track of the row)
	j = 0 #index for inner loop (keeps track of the column)
	k = 0 #index for data (never gets reset)

	while (i < height) and (k < len(msg.data)): #go through all rows (starting at the top (0,0))
		j = 0 #reset index (start at the start of the new row)

		while (j < width) and (k < len(msg.data)): #go through a single row
			if msg.data[k] > 99: #check if its an obstacle
				wallList.append((j,i)) #add to the list of obstacles

			j+=1
			k+=1

		i+=1

	grid.walls = wallList

def getStart(msg):
	global startX
	global startY
	global offSetX
	global offSetY
	global resolution

	startX = int(round((((msg.pose.pose.position.x - offSetX)/resolution) - 0.5), 0))
	startY = int(round((((msg.pose.pose.position.y - offSetY)/resolution) - 0.5), 0))

def publishGridCells(path):
	global seqNum
	global offSetX
	global offSetY
	global resolution
	global gridCellsPub

	head = Header()
	head.seq = seqNum
	seqNum += 1
	head.stamp = rospy.get_rostime()
	head.frame_id = "map"

	points = pointList(path)#get the points
	
	gridCells = GridCells()
	#fill the message with the necessary data
	gridCells.header = head
	gridCells.cell_width = resolution
	gridCells.cell_height = resolution
	gridCells.cells = points
	gridCellsPub.publish(gridCells)

def pointList(path): #creates a list of points from a list of tuples (x,y)
	points = []
	for i in path:
		newPoint = Point()

		newPoint.x = i[0]#*resolution
		newPoint.y = i[1]#*resolution
		newPoint.z = 0

		points.append(newPoint)

	return points

def scalePath(path):
	global offSetX
	global offSetY
	global resolution

	pathScaled = []

	for i in path:
		newX = round((i[0] + 0.5)*resolution + offSetX, 5)
		newY = round((i[1] + 0.5)*resolution + offSetY, 5)
		pathScaled.append((newX,newY))

	return pathScaled

def expandWalls(walls): #takes a list of ruples, publishes and returns a list of expanded walls (also a list of tuples)
	global seqNum
	global offSetX
	global offSetY
	global resolution
	global expandedWallsPub
	expansionList = []
	scaledExpansionList = []
	expandedWallList = []

	#for each wall cell make the cells next to it impassible as well
	for i in walls: 
		if ((i[0]+1),i[1]) not in expansionList:
			if ((i[0]+1),i[1]) not in walls:
				expansionList.append(((i[0]+1),i[1]))
		
		if ((i[0]-1),i[1]) not in expansionList:
			if ((i[0]-1),i[1]) not in walls:
				expansionList.append(((i[0]-1),i[1]))
		
		if (i[0],(i[1]+1)) not in expansionList:
			if (i[0],(i[1]+1)) not in walls:
				expansionList.append((i[0],(i[1]+1)))
		
		if (i[0],(i[1]-1)) not in expansionList:
			if (i[0],(i[1]-1)) not in walls:
				expansionList.append((i[0],(i[1]-1)))

		#Diagonals:
		if (i[0]+1,(i[1]+1)) not in expansionList:
			if (i[0]+1,(i[1]+1)) not in walls:
				expansionList.append((i[0]+1,(i[1]+1)))

		if (i[0]+1,(i[1]-1)) not in expansionList:
			if (i[0]+1,(i[1]-1)) not in walls:
				expansionList.append((i[0]+1,(i[1]-1)))

		if (i[0]-1,(i[1]+1)) not in expansionList:
			if (i[0]-1,(i[1]+1)) not in walls:
				expansionList.append((i[0]-1,(i[1]+1)))

		if (i[0]-1,(i[1]-1)) not in expansionList:
			if (i[0]-1,(i[1]-1)) not in walls:
				expansionList.append((i[0]-1,(i[1]-1)))

	for i in walls:
		expandedWallList.append((i[0],i[1]))

	for i in expansionList:
		expandedWallList.append((i[0],i[1]))

	for i in expansionList:
		newX = round((i[0] + 0.5)*resolution + offSetX, 5)
		newY = round((i[1] + 0.5)*resolution + offSetY, 5)
		scaledExpansionList.append((newX,newY))

	head = Header()
	head.seq = seqNum
	seqNum += 1
	head.stamp = rospy.get_rostime()
	head.frame_id = "map"

	points = pointList(scaledExpansionList)#get the points
	
	gridCells = GridCells()
	gridCells.header = head
	gridCells.cell_width = resolution
	gridCells.cell_height = resolution
	gridCells.cells = points
	expandedWallsPub.publish(gridCells)
	return expandedWallList

def callAStar(msg):
	global grid
	global startX
	global startY
	global goalX
	global goalY
	global offSetX
	global offSetY
	global resolution
	global height
	global width

	goalX = int(round((((msg.pose.position.x - offSetX)/resolution) - 0.5), 0))
	goalY = int(round((((msg.pose.position.y - offSetY)/resolution) - 0.5), 0))
	start = (startX, startY)
	goal = (goalX, goalY)
	expandedGrid = GridWithWeights(width, height)
	expandedGrid.walls = expandWalls(grid.walls)
	came_from, cost_so_far = a_star_search(expandedGrid, start, goal)
	path = reconstruct_path(came_from, start, goal)
	pathScaled = scalePath(path)
	publishGridCells(pathScaled)
	publishPath(pathScaled)

def publishPath(cells): #takes a list of cells in the order that we wish to visit them and publishes a path message
	 global pathPub

	 print "Publish Path, calling get_Path"
	 #path = get_Path(cells)
	 get_Path(cells)
	 print "Returned from get_Path"
	 #pathPub.publish(path)
	
def get_Path(cells): #takes a list of location tuples in the order that we wish to visit them and returns a path message
	print "Get Path"
	global seqNum

	#create header:
	pathHead = Header()
	pathHead.seq = seqNum
	seqNum += 1
	pathHead.stamp = rospy.get_rostime()
	pathHead.frame_id = "aStar_Path"
	
	#waypoints = cells #Assume all cells in the path are waypoints until proven otherwise
	waypoints = []
	poses = [] #create the list to store all of the poses (waypoints) as PoseStamped objects

	i = 1
	while i < len(waypoints)-1:#we don't want to check the first or last cels
		#if the waypoint has the same x or y as the ones in front and behind it, no turn is needed, therefore it is not a waypoint
		if not ((waypoints[i-1])[0] == (waypoints[i])[0] == (waypoints[i+1])[0] or (waypoints[i-1])[1] == (waypoints[i])[1] == (waypoints[i+1])[1]):
			waypoints.append(waypoints[i])#remove the cell from the list of waypoints
			print "Append"
		i = i+1
	print "WAYPOINTS"
	print waypoints
	print " "
	print "CELLS"
	print cells
		#convert that angle to a quaternian:
		#quaternion = tf.transformations.quaternion_from_euler(0, 0, turn)
		#pose = Pose()
		#pose.orientation.x = quaternion[0]
		#pose.orientation.y = quaternion[1]
		#pose.orientation.z = quaternion[2]
		#pose.orientation.w = quaternion[3]
		
		#set the coordinates (may need conversion later to go from grid coordinates to actual)
		#pose.position.x = cells[i].x
		#pose.position.y = cells[i].y
		#pose.position.z = 0

		#create header:
		#head = Header()
		#head.seq = seqNum
		#seqNum += 1
		#head.stamp = rospy.get_rostime()
		#head.frame_id = "waypoint"
			
		#poseStamped = PoseStamped(head, pose) #create the PoseStamped object

		#poses.append(poseStamped)
	
	#return Path(pathHead,poses)


if __name__ == '__main__':
	rospy.init_node('aStar')
	global gridCellsPub
	global expandedWallsPub
	global seqNum
	seqNum = 0

	map_sub = rospy.Subscriber('/map', OccupancyGrid, getMap, queue_size=1)
	start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, getStart, queue_size=1)
	goal_sub = rospy.Subscriber('/goal', PoseStamped, callAStar, queue_size=1)
	gridCellsPub = rospy.Publisher('aStar_Closed', GridCells, queue_size=10)
	expandedWallsPub = rospy.Publisher('walls',GridCells, queue_size=10)
	pathPub = rospy.Publisher('aStar_Path', Path, queue_size=10)

	rospy.sleep(rospy.Duration(1, 0))
	while(not rospy.is_shutdown()):
		rospy.sleep(0.15)

