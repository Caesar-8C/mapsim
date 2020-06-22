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
		for node in self.map.G.nodes:
			dist = tupleDistance(self.map.G.nodes[node]['coordinates'], pos)
			if dist < self.map.NODE_SIZE:
				self.node = node # TODO fix
				self.edge = None
				return True

		for edge in self.map.G.edges:
			x1, y1 = self.map.G.nodes[edge[0]]['coordinates']
			x2, y2 = self.map.G.nodes[edge[1]]['coordinates']
			halfWidth = max(self.map.G.edges[edge[0], edge[1]]['width']/2, self.map.MIN_CLICKABLE_CORRIDOR_WIDTH/2)

			angle = np.arctan2(x1-x2, y2-y1)
			x3 = x1 + np.cos(angle)*halfWidth
			y3 = y1 + np.sin(angle)*halfWidth

			x4 = x1 - np.cos(angle)*halfWidth
			y4 = y1 - np.sin(angle)*halfWidth

			x6 = x2 + np.cos(angle)*halfWidth
			y6 = y2 + np.sin(angle)*halfWidth

			AM = tupleSubtract(pos, (x3, y3))
			AB = tupleSubtract((x4, y4), (x3, y3))
			AD = tupleSubtract((x6, y6), (x3, y3))

			if 0<np.dot(AM,AB) and np.dot(AM,AB)<np.dot(AB,AB) and 0<np.dot(AM,AD) and np.dot(AM,AD)<np.dot(AD,AD):
				self.node = None
				self.edge = edge
				return True

		return False


	def reset(self):
		self.x, self.y = self.resetCoordinates
		self.theta = 0
		self.velocity = [0, 0, 0]
