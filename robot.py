import numpy as np
from utils import tupleDistance, tupleSubtract

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
		if not self.map.bridge.enabled:
			self.x += self.velocity[0]*np.cos(self.theta) - self.velocity[1]*np.sin(self.theta)
			self.y += self.velocity[0]*np.sin(self.theta) + self.velocity[1]*np.cos(self.theta)
			self.theta += self.velocity[2]*np.pi/180.

			for i in range(3):
				self.velocity[i] = self.changeVelocity(self.velocity[i], controlAction[i], self.maxVelocity[i])

		# UNCOMMENT FOR COLLISION CHECKING
		# if not self.obstacleCollisionCheck():
		# 	self.reset()
		# for i in np.linspace(0, 2*np.pi, num=self.COLLISION_DISCRETIZATION):
		# 	x = self.x + np.cos(i)*self.map.ROBOT_SIZE
		# 	y = self.y + np.sin(i)*self.map.ROBOT_SIZE
		# 	if not self.mapCollisionCheck((x, y)):
		# 		self.reset()
		# 		break

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