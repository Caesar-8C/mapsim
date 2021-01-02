import numpy as np
import time

def tupleSum(t1, t2):
	return (t1[0] + t2[0], t1[1] + t2[1])

def tupleSubtract(t1, t2):
	return (t1[0] - t2[0], t1[1] - t2[1])

def tupleDistance(t1, t2):
	return np.linalg.norm(tupleSubtract(t1, t2))

def between(midNumber, boundary1, boundary2):
	if boundary1 > boundary2:
		if boundary1 > midNumber and midNumber > boundary2:
			return True
	else:
		if boundary2 > midNumber and midNumber > boundary1:
			return True
	return False

class Agent:

	def __init__(self, map, mapIndex, current_node=None, goal_node=None, lane = None, speed=None):
		self.NEARBY_LIMIT = 120
		self.SAFETY_LIMIT = 10
		self.RANDOM_NORMAL_CHANGE_SPEED = 0.0005
		self.RANDOM_NORMAL_CHANGE_LANE = 0.0005
		self.RANDOM_NORMAL_STOP = 0.05
		self.LANE_REACHED = 2
		self.fastforward = 1

		self.size = 0
		self.path = []
		self.radius = 10
		self.stop = False
		self.map = map
		self.mapIndex = mapIndex

		self.stopTime = None
		self.waitTime = None


		self.current_node = current_node
		self.goal_node = goal_node
		self.forwardSpeed = speed

		if self.current_node is None:
			self.chooseOrigin()
		if self.goal_node is None:
			self.chooseGoal()
		if self.forwardSpeed is None:
			self.chooseSpeed()

		self.x, self.y = self.map.G.nodes[self.current_node]['coordinates']
		self.coordinates = (self.x, self.y)

		self.calculatePath()
		self.next_node = self.path[1]


		self.sideSpeed = 0
		self.lane = None
		self.targetLane = None
		self.direction = 1 if self.map.G.edges[self.current_node, self.next_node]['start'] == self.current_node else -1
		self.maxdist = self.map.G.edges[self.current_node, self.next_node]['length']
		self.distance = 0 if self.direction == 1 else self.maxdist
		self.distanceAcross = self.map.G.edges[self.current_node, self.next_node]['width']/2

		self.changeLane()
		self.setTargetLane(1)


	def move(self):
		if self.stop:
			if self.stopTime is None:
				self.startTimer()
			elif time.time() > self.stopTime + self.waitTime():
				self.stopTimer()
			return

		someoneInTheWay, busyLanes, danger = self.checkForAnyoneNearby()

		lanes = range(self.map.G.edges[self.current_node, self.next_node]['lanes'])
		safeLanes = [i for i in lanes if i not in busyLanes]

		if someoneInTheWay and danger:
			self.forwardSpeed = 0
		elif self.forwardSpeed == 0:
			self.chooseSpeed()

		if not someoneInTheWay:
			self.moveNormal()
		elif len(safeLanes) == 0:
			self.forwardSpeed = 0
		else:
			self.moveAvoid(safeLanes)


	def startTimer(self):
		self.stopTime = time.time()
		self.waitTime = np.random.random()*10+5

	def stopTimer(self):
		r = np.random.random()
		if r < 0.4:
			self.chooseGoal()
			self.calculatePath()
		self.stop = False

	def checkForAnyoneNearby(self):
		edge = [self.current_node, self.next_node]
		nearbyAgents = []
		busyLanes = []
		danger = False
		for agentIndex in self.map.agents:
			agent = self.map.agents[agentIndex]
			if agent.current_node in edge and agent.next_node in edge\
			and between(agent.distance, self.distance, self.distance+self.NEARBY_LIMIT*self.direction):
				nearbyAgents.append(agentIndex)
				if not agent.lane in busyLanes:
					busyLanes.append(agent.lane)
				if between(agent.distance, self.distance, self.distance+self.SAFETY_LIMIT*self.direction):
					danger = True
		someoneInTheWay = True if self.lane in busyLanes else False
		return someoneInTheWay, busyLanes, danger

	def chooseOrigin(self):
		self.current_node = list(self.map.G.nodes)[np.random.randint(len(self.map.G.nodes))]

	def chooseGoal(self):
		while True:
			self.goal_node = list(self.map.G.nodes)[np.random.randint(len(self.map.G.nodes))]
			if self.goal_node != self.current_node:
				break

	def calculatePath(self):
		self.path = self.map.pathMap[self.current_node][self.goal_node].copy()

	def chooseSpeed(self):
		self.forwardSpeed = np.random.random()*20+40

	def moveNormal(self):
		r = np.random.random()
		if r < self.RANDOM_NORMAL_CHANGE_SPEED:
			self.chooseSpeed()
		r = np.random.random()
		if r < self.RANDOM_NORMAL_CHANGE_LANE:
			r = np.random.randint(self.map.G.edges[self.current_node, self.next_node]['lanes'])
			self.setTargetLane(r)
		r = np.random.random()
		if r < self.RANDOM_NORMAL_STOP:
			self.startTimer()
		self.step()

	def moveAvoid(self, safeLanes):
		lowerLanes = []
		higherLanes = []
		targetLane = None

		for lane in safeLanes:
			if lane < self.lane:
				lowerLanes.append(lane)
			if lane > self.lane:
				higherLanes.append(lane)

		if self.direction > 0:
			if len(higherLanes) > 0:
				targetLane = min(higherLanes)
		else:
			if len(lowerLanes) > 0:
				targetLane = max(lowerLanes)


		if not targetLane is None:
			self.setTargetLane(targetLane)

		self.step()

	def setTargetLane(self, targetLane):
		self.targetLane = targetLane
		self.sideSpeed = np.sign(self.targetLane - self.lane)*20

	def step(self):
		sinAcross = self.map.G.edges[self.current_node, self.next_node]['sinAcross']
		cosAcross = self.map.G.edges[self.current_node, self.next_node]['cosAcross']
		sinAlong = self.map.G.edges[self.current_node, self.next_node]['sinAlong']
		cosAlong = self.map.G.edges[self.current_node, self.next_node]['cosAlong']
		coords = list(self.map.G.nodes[self.map.G.edges[self.current_node, self.next_node]['start']]['coordinates'])
		coords[0] += cosAcross*self.map.G.edges[self.current_node, self.next_node]['width']/2
		coords[1] += sinAcross*self.map.G.edges[self.current_node, self.next_node]['width']/2

		distIncrementAlong = self.direction*self.forwardSpeed*self.fastforward/self.map.fps
		distIncrementAcross = self.sideSpeed*self.fastforward/self.map.fps

		self.distance += distIncrementAlong
		self.distanceAcross += distIncrementAcross
		self.x = coords[0] - cosAcross*self.distanceAcross - cosAlong*self.distance
		self.y = coords[1] - sinAcross*self.distanceAcross - sinAlong*self.distance
		self.coordinates = (self.x, self.y)

		self.changeLane()
		self.targetLaneReached()

		if 	   (self.direction > 0 and self.distance >= self.maxdist)\
			or (self.direction < 0 and self.distance <= 0):
			self.corridorEndReached()

	def changeLane(self):
		laneWidth = self.map.G.edges[self.current_node, self.next_node]['laneWidth']
		lanes = np.array(range(self.map.G.edges[self.current_node, self.next_node]['lanes']))
		lanes = np.abs(lanes*laneWidth + laneWidth/2 - self.distanceAcross)
		self.lane = np.argmin(lanes)

	def targetLaneReached(self):
		if self.targetLane is None: return
		laneWidth = self.map.G.edges[self.current_node, self.next_node]['laneWidth']

		if (self.distanceAcross - self.targetLane*laneWidth - laneWidth/2)*self.sideSpeed > 0:
			self.sideSpeed *= -0.8

		if np.abs(self.distanceAcross - self.targetLane*laneWidth - laneWidth/2) < self.LANE_REACHED:
			self.targetLane = None
			self.sideSpeed = 0

	def corridorEndReached(self):
		if self.next_node == self.goal_node:
			self.goalReached()
		else:
			del self.path[0]
			self.current_node = self.path[0]
			self.next_node = self.path[1]

			start_node = self.map.G.edges[self.current_node, self.next_node]['start']
			self.direction = 1 if self.current_node == start_node else -1

			lane = self.findClosestLane()
			self.setTargetLane(lane)

			self.maxdist = self.map.G.edges[self.current_node, self.next_node]['length']



			agent = np.array(self.coordinates)

			origin = np.array(self.map.G.nodes[start_node]['coordinates'])
			sinAcross = self.map.G.edges[self.current_node, self.next_node]['sinAcross']
			cosAcross = self.map.G.edges[self.current_node, self.next_node]['cosAcross']
			width = self.map.G.edges[self.current_node, self.next_node]['width']
			origin[0] += cosAcross*width/2
			origin[1] += sinAcross*width/2
			agent -= origin

			rotAngle = -np.pi - self.map.G.edges[self.current_node, self.next_node]['angleAlong']

			self.distance = agent[0]*np.cos(rotAngle) - agent[1]*np.sin(rotAngle)
			self.distanceAcross = agent[0]*np.sin(rotAngle) + agent[1]*np.cos(rotAngle)
			pass


	def findClosestLane(self):
		width = self.map.G.edges[self.current_node, self.next_node]['width']
		laneWidth = self.map.G.edges[self.current_node, self.next_node]['laneWidth']
		x, y = self.map.G.nodes[self.current_node]['coordinates']
		sinAcross = self.map.G.edges[self.current_node, self.next_node]['sinAcross']
		cosAcross = self.map.G.edges[self.current_node, self.next_node]['cosAcross']
		laneNum = self.map.G.edges[self.current_node, self.next_node]['lanes']

		lanes = np.array(range(laneNum))
		lanes = lanes*laneWidth + laneWidth/2. - width/2.
		lanes *= -1
		lanes_x = [x + self.direction*cosAcross*lane for lane in lanes]
		lanes_y = [y + self.direction*sinAcross*lane for lane in lanes]

		norms = [np.linalg.norm([lanes_x[i] - self.x, lanes_y[i] - self.y]) for i in range(laneNum)]

		return np.argmin(norms)

	def goalReached(self):
		del self.map.agents[self.mapIndex]

class Room:
	def __init__(self, start, end, distance):
		self.start = start
		self.end = end
		self.distance = distance

class Robot:
	def __init__(self, x, y, map):
		self.resetCoordinates = (x, y)
		self.x = x
		self.y = y
		self.theta = 0
		self.distance = None
		self.lane = None

		self.node = None
		self.edge = None
		self.edge_candidates = None

		self.map = map
		self.velocity = [0, 0, 0]
		self.maxVelocity = [100./self.map.fps, 100./self.map.fps, 150./self.map.fps] # pixels per second

		self.COLLISION_DISCRETIZATION = 13


	def move(self, controlAction):
		self.x += self.velocity[0]*np.cos(self.theta) - self.velocity[1]*np.sin(self.theta)
		self.y += self.velocity[0]*np.sin(self.theta) + self.velocity[1]*np.cos(self.theta)
		self.theta += self.velocity[2]*np.pi/180.

		for i in range(3):
			self.velocity[i] = self.changeVelocity(self.velocity[i], controlAction[i], self.maxVelocity[i])

		if not self.obstacleCollisionCheck():
			self.reset()

		for i in np.linspace(0, 2*np.pi, num=self.COLLISION_DISCRETIZATION):
			x = self.x + np.cos(i)*self.map.ROBOT_SIZE
			y = self.y + np.sin(i)*self.map.ROBOT_SIZE
			if not self.mapCollisionCheck((x, y)):
				self.reset()
				break

		self.mapLocate()


	def changeVelocity(self, velocity, controlAction, maxVelocity):
		velocity += controlAction - np.sign(velocity)*1
		if controlAction == 0 and abs(velocity) < 1:
			velocity = 0
		if abs(velocity) > maxVelocity:
			return np.sign(velocity) * maxVelocity
		return velocity

	def obstacleCollisionCheck(self):
		pos = (self.x, self.y)
		for agent in self.map.agents:
			agent_pos = (self.map.agents[agent].x, self.map.agents[agent].y)
			dist = tupleDistance(agent_pos, pos)
			if dist < self.map.agents[agent].size + self.map.ROBOT_SIZE:
				return False
		return True

	def mapCollisionCheck(self, pos):
		if self.map.insideNodeCheck(pos, self.map.NODE_SIZE) != None:
			return True
		if len(self.map.insideEdgeCheck(pos)) > 0:
			return True
		return False


	def mapLocate(self):
		pos = (self.x, self.y)

		self.node = None
		self.setEdge(None)
		self.edge_candidates = []

		self.node = self.map.insideNodeCheck(pos, self.map.NODE_SIZE)
		self.edge_candidates = self.map.insideEdgeCheck(pos)

		if self.node == None and len(self.edge_candidates) == 1:
			self.setEdge(self.edge_candidates[0])

		if self.node == None and self.edge == None and len(self.edge_candidates) > 1:
			for node in self.edge_candidates[0]:
				if node in self.edge_candidates[1]:
					self.node = node
					break


	def setEdge(self, edge):
		if edge == None:
			self.edge = None
			self.distance = None
		else:
			self.edge = edge
			self.distance = self.computeCorridorDistance()


	def computeCorridorDistance(self):
		R = (self.x, self.y)
		A = self.map.G.nodes[self.edge[0]]['coordinates']
		B = self.map.G.nodes[self.edge[1]]['coordinates']
		dist = self.map.G.edges[self.edge]['length']

		AR = tupleSubtract(R, A)
		AB = tupleSubtract(B, A)
		return int(np.dot(AR, AB)/dist)


	def reset(self):
		self.x, self.y = self.resetCoordinates
		self.theta = 0
		self.velocity = [0, 0, 0]
