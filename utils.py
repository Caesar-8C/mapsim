import numpy as np

def tupleSum(t1, t2):
	return (t1[0] + t2[0], t1[1] + t2[1])

def tupleSubtract(t1, t2):
	return (t1[0] - t2[0], t1[1] - t2[1])

def tupleDistance(t1, t2):
	return np.linalg.norm(tupleSubtract(t1, t2))

def between(a, b, c):
	if b > c:
		if b > a and a > c:
			return True
	else:
		if c > a and a > b:
			return True
	return False

class Agent:
	def __init__(self, origin_node, end_node, lane, speed):
		self.x = 0
		self.y = 0
		self.size = 0

		self.origin_node = origin_node
		self.end_node = end_node
		self.lane = lane
		self.speed = speed
		self.distance = 0

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
		self.edge = None
		self.edge_candidates = []

		self.node = self.map.insideNodeCheck(pos, self.map.NODE_SIZE)
		self.edge_candidates = self.map.insideEdgeCheck(pos)

		if self.node == None and len(self.edge_candidates) == 1:
			self.edge = self.edge_candidates[0]


	def reset(self):
		self.x, self.y = self.resetCoordinates
		self.theta = 0
		self.velocity = [0, 0, 0]
