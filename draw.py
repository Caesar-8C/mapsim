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


def drawCorridor(self, node, successor):
	x1 = self.G.nodes[node]['coordinates'][0]
	y1 = self.G.nodes[node]['coordinates'][1]
	x2 = self.G.nodes[successor]['coordinates'][0]
	y2 = self.G.nodes[successor]['coordinates'][1]
	halfWidth = self.G.edges[node, successor]['width']/2

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

def drawLanes(self, node, successor):
	x1 = self.G.nodes[node]['coordinates'][0]
	y1 = self.G.nodes[node]['coordinates'][1]
	x2 = self.G.nodes[successor]['coordinates'][0]
	y2 = self.G.nodes[successor]['coordinates'][1]
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

def drawRooms(self, node, successor):
	x1 = self.G.nodes[node]['coordinates'][0]
	y1 = self.G.nodes[node]['coordinates'][1]
	x2 = self.G.nodes[successor]['coordinates'][0]
	y2 = self.G.nodes[successor]['coordinates'][1]
	width = self.G.edges[node, successor]['width']

	angleAcross = np.arctan2(x1-x2, y2-y1)
	angleAlong = np.arctan2(y1-y2, x1-x2)
	for room in self.G.edges[node, successor]['rooms']:
		dist = self.G.edges[node, successor]['rooms'][room]
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
		x1 = self.G.nodes[agent.origin_node]['coordinates'][0]
		y1 = self.G.nodes[agent.origin_node]['coordinates'][1]
		x2 = self.G.nodes[agent.end_node]['coordinates'][0]
		y2 = self.G.nodes[agent.end_node]['coordinates'][1]
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


def drawSuccessors(self, node, visited):
	visited.append(node)
	for successor in list(self.G.successors(node)):
		self.drawCorridor(node, successor)
		self.drawRooms(node, successor)
		self.drawLanes(node, successor)
		if successor not in visited:
			self.drawSuccessors(successor, visited)

	self.draw.circle(self.screen, self.NODE_COLOR, self.G.nodes[node]['coordinates'], self.NODE_SIZE)