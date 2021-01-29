import numpy as np
from utils import tupleDistance, tupleSubtract, normalizeAngle

class Robot:
	def __init__(self, x, y, theta, map):
		self.resetCoordinates = (x, y)
		self.x = x
		self.y = y
		self.theta = theta
		self.distance = None
		self.lane = None

		self.node = None
		self.edge = None
		self.edge_candidates = None

		self.map = map
		self.velocity = [0, 0, 0]
		self.maxVelocity = [100./self.map.fps, 100./self.map.fps, np.pi/self.map.fps] # pixels per second

		self.automoveEnabled = False
		self.stopAutoMove = False

		self.COLLISION_DISCRETIZATION = 13


	def move(self, controlAction):
		if not self.map.bridge.enabled or not self.automoveEnabled:
			self.x += self.velocity[0]*np.cos(self.theta) - self.velocity[1]*np.sin(self.theta)
			self.y += self.velocity[0]*np.sin(self.theta) + self.velocity[1]*np.cos(self.theta)
			self.theta += self.velocity[2]

			for i in range(3):
				self.velocity[i] = self.changeVelocity(self.velocity[i], controlAction[i], self.maxVelocity[i])

		self.mapLocate()


	def automove(self, waypoint):
		if not self.stopAutoMove:
			angle = np.arctan2(self.y-waypoint[1], self.x-waypoint[0])+np.pi
			self.x += self.maxVelocity[0]*np.cos(angle)
			self.y += self.maxVelocity[0]*np.sin(angle)
			angle = normalizeAngle(angle)
			robotAngle = normalizeAngle(self.theta)
			self.theta += self.maxVelocity[2]*np.sign(angle-robotAngle)
		self.mapLocate()


	def changeVelocity(self, velocity, controlAction, maxVelocity):
		velocity += controlAction - np.sign(velocity)*1
		if controlAction == 0 and abs(velocity) < 1:
			velocity = 0
		if abs(velocity) > maxVelocity:
			return np.sign(velocity) * maxVelocity
		return velocity


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
		# R = (self.x, self.y)
		# A = self.map.G.nodes[self.edge[0]]['coordinates']
		# B = self.map.G.nodes[self.edge[1]]['coordinates']
		# dist = self.map.G.edges[self.edge]['length']
		#
		# AR = tupleSubtract(R, A)
		# AB = tupleSubtract(B, A)
		# return int(np.dot(AR, AB)/dist)

		start_node = self.map.G.edges[self.edge]['start']
		coordinates = [self.x, self.y]
		origin = np.array(self.map.G.nodes[start_node]['coordinates'])
		sinAcross = self.map.G.edges[self.edge]['sinAcross']
		cosAcross = self.map.G.edges[self.edge]['cosAcross']
		width = self.map.G.edges[self.edge]['width']
		origin[0] += cosAcross*width/2
		origin[1] += sinAcross*width/2
		coordinates -= origin

		rotAngle = -np.pi - self.map.G.edges[self.edge]['angleAlong']

		distance = coordinates[0]*np.cos(rotAngle) - coordinates[1]*np.sin(rotAngle)
		return distance


	def reset(self):
		self.x, self.y = self.resetCoordinates
		self.theta = 0
		self.velocity = [0, 0, 0]

	def enableAutomove(self):
		self.automoveEnabled = True

	def disableAutomove(self):
		self.automoveEnabled = False