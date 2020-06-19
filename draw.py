import pygame as pyg
import numpy as np

class Draw:
	def __init__(self, scale=1.5, shift=(0, 0)):
		self.scale = scale
		self.shift = shift

	def line(self, surface, color, start_pos, end_pos, width):
		start_pos = (int((start_pos[0]+self.shift[0]) * self.scale), int((start_pos[1]+self.shift[1]) * self.scale))
		end_pos = (int((end_pos[0]+self.shift[0]) * self.scale), int((end_pos[1]+self.shift[1]) * self.scale))
		pyg.draw.line(surface, color, start_pos, end_pos, width)

	def circle(self, surface, color, center, radius):
		center = (int((center[0]+self.shift[0]) * self.scale), int((center[1]+self.shift[1]) * self.scale))
		radius = int(radius*self.scale)
		pyg.draw.circle(surface, color, center, radius)

	def polygon(self, surface, color, points):
		p = []
		for point in points:
			p.append((int((point[0]+self.shift[0]) * self.scale), int((point[1]+self.shift[1]) * self.scale)))
		pyg.draw.polygon(surface, color, tuple(p))


def drawNodes(self, color=None, radius=None):
	if color == None:
		color = self.NODE_COLOR
	if radius == None:
		radius = self.NODE_SIZE

	for node in self.G.nodes:
		self.draw.circle(self.screen, color, self.G.nodes[node]['coordinates'], radius)

def drawCorridors(self):
	for corridor in self.G.edges:
		x1, y1 = self.G.nodes[corridor[0]]['coordinates']
		x2, y2 = self.G.nodes[corridor[1]]['coordinates']
		halfWidth = self.G.edges[corridor[0], corridor[1]]['width']/2

		angle = np.arctan2(x1-x2, y2-y1)
		x3 = x1 + np.cos(angle)*halfWidth
		y3 = y1 + np.sin(angle)*halfWidth

		x4 = x1 - np.cos(angle)*halfWidth
		y4 = y1 - np.sin(angle)*halfWidth

		x5 = x2 - np.cos(angle)*halfWidth
		y5 = y2 - np.sin(angle)*halfWidth

		x6 = x2 + np.cos(angle)*halfWidth
		y6 = y2 + np.sin(angle)*halfWidth

		self.draw.polygon(self.screen, self.CORRIDOR_COLOR, ((x3, y3), (x4, y4), (x5, y5), (x6, y6)))
		self.drawLanes(corridor[0], corridor[1])

def drawLanes(self, node, successor):
	x1, y1 = self.G.nodes[node]['coordinates']
	x2, y2 = self.G.nodes[successor]['coordinates']
	width = self.G.edges[node, successor]['width']
	
	lanes = int(width/self.LANE_PIXEL_WIDTH)
	if lanes == 0:
		lanes = 1
	self.G.edges[node, successor]['lanes'] = lanes
	laneWidth = width/lanes
	self.G.edges[node, successor]['laneWidth'] = laneWidth

	if lanes > 1:
		angleNorm = np.arctan2(x1-x2, y2-y1)
		angleCos = np.cos(angleNorm)
		angleSin = np.sin(angleNorm)

		x3 = x1 + angleCos*width/2
		y3 = y1 + angleSin*width/2

		x4 = x2 + angleCos*width/2
		y4 = y2 + angleSin*width/2

		xStep = angleCos*laneWidth
		yStep = angleSin*laneWidth

		for i in range(lanes-1):
			x3 -= xStep
			y3 -= yStep

			x4 -= xStep
			y4 -= yStep

			self.draw.line(self.screen, self.LANE_COLOR, (x3, y3), (x4, y4), 1)

def drawRooms(self):
	for roomNumber in self.rooms:
		x1, y1 = self.G.nodes[self.rooms[roomNumber].start]['coordinates']
		x2, y2 = self.G.nodes[self.rooms[roomNumber].end]['coordinates']
		width = self.G.edges[self.rooms[roomNumber].start, self.rooms[roomNumber].end]['width']

		angleAcross = np.arctan2(x1-x2, y2-y1)
		angleAlong = np.arctan2(y1-y2, x1-x2)
		dist = self.rooms[roomNumber].distance
		x3 = x1 + np.cos(angleAlong)*dist - np.sign(dist)*np.cos(angleAcross)*width/2
		y3 = y1 + np.sin(angleAlong)*dist - np.sign(dist)*np.sin(angleAcross)*width/2

		x4 = x3 + np.cos(angleAlong)*20
		y4 = y3 + np.sin(angleAlong)*20

		self.draw.line(self.screen, self.ROOM_COLOR, (x3, y3), (x4, y4), 3)

def drawAgents(self):
	indicesToDelete = []
	for agentIndex in self.agents:
		#TODO add lane existence check
		#TODO add negative speed check
		agent = self.agents[agentIndex]
		x1, y1 = self.G.nodes[agent.origin_node]['coordinates']
		x2, y2 = self.G.nodes[agent.end_node]['coordinates']
		laneWidth = self.G.edges[agent.origin_node, agent.end_node]['laneWidth']
		width = self.G.edges[agent.origin_node, agent.end_node]['width']

		angleAcross = np.arctan2(x1-x2, y2-y1)
		angleAlong = np.arctan2(y1-y2, x1-x2)

		x3 = int(np.cos(angleAcross)*(width/2 - agent.lane*laneWidth - laneWidth/2) - np.cos(angleAlong)*agent.distance)
		y3 = int(np.sin(angleAcross)*(width/2 - agent.lane*laneWidth - laneWidth/2) - np.sin(angleAlong)*agent.distance)

		if agent.speed > 0:
			x3 += x1
			y3 += y1
		else:
			x3 += x2
			y3 += y2

		self.draw.circle(self.screen, self.AGENT_COLOR, (x3, y3), int(laneWidth/2))

		agent.distance += agent.speed/self.fps

		maxdist = np.linalg.norm((y2-y1, x2-x1))

		if np.abs(agent.distance) >= maxdist:
			indicesToDelete.append(agentIndex)

	for i in indicesToDelete:
		del self.agents[i]

def drawRobot(self):
	self.draw.circle(self.screen, self.ROBOT_COLOR, (self.robot.x, self.robot.y), self.ROBOT_SIZE)
	robotFront = (self.robot.x + np.cos(self.robot.theta)*self.ROBOT_SIZE, self.robot.y + np.sin(self.robot.theta)*self.ROBOT_SIZE)
	self.draw.line(self.screen, self.LANE_COLOR, (self.robot.x, self.robot.y), robotFront, 1)