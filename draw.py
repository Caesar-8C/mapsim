import pygame as pyg
import numpy as np

class Draw:
	def __init__(self, scale=1.5, shift=(0, 0)):
		self.scale = scale
		self.shift = shift

	def line(self, surface, color, start_pos, end_pos, width=1):
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

		sinAcross = self.G.edges[corridor[0], corridor[1]]['sinAcross']
		cosAcross = self.G.edges[corridor[0], corridor[1]]['cosAcross']

		x3 = x1 + cosAcross*halfWidth
		y3 = y1 + sinAcross*halfWidth

		x4 = x1 - cosAcross*halfWidth
		y4 = y1 - sinAcross*halfWidth

		x5 = x2 - cosAcross*halfWidth
		y5 = y2 - sinAcross*halfWidth

		x6 = x2 + cosAcross*halfWidth
		y6 = y2 + sinAcross*halfWidth

		self.draw.polygon(self.screen, self.CORRIDOR_COLOR, ((x3, y3), (x4, y4), (x5, y5), (x6, y6)))
		self.drawLanes(corridor[0], corridor[1])

def drawLanes(self, node, successor):
	x1, y1 = self.G.nodes[node]['coordinates']
	x2, y2 = self.G.nodes[successor]['coordinates']
	width = self.G.edges[node, successor]['width']

	lanes = self.G.edges[node, successor]['lanes']
	laneWidth = self.G.edges[node, successor]['laneWidth']

	if lanes > 1:
		sinAcross = self.G.edges[node, successor]['sinAcross']
		cosAcross = self.G.edges[node, successor]['cosAcross']

		x3 = x1 + cosAcross*width/2
		y3 = y1 + sinAcross*width/2

		x4 = x2 + cosAcross*width/2
		y4 = y2 + sinAcross*width/2

		xStep = cosAcross*laneWidth
		yStep = sinAcross*laneWidth

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

		sinAcross = self.G.edges[self.rooms[roomNumber].start, self.rooms[roomNumber].end]['sinAcross']
		cosAcross = self.G.edges[self.rooms[roomNumber].start, self.rooms[roomNumber].end]['cosAcross']
		sinAlong = self.G.edges[self.rooms[roomNumber].start, self.rooms[roomNumber].end]['sinAlong']
		cosAlong = self.G.edges[self.rooms[roomNumber].start, self.rooms[roomNumber].end]['cosAlong']

		dist = self.rooms[roomNumber].distance
		x3 = x1 + cosAlong*(dist-10) - np.sign(dist)*cosAcross*width/2
		y3 = y1 + sinAlong*(dist-10) - np.sign(dist)*sinAcross*width/2

		x4 = x3 + cosAlong*20
		y4 = y3 + sinAlong*20

		self.draw.line(self.screen, self.ROOM_COLOR, (x3, y3), (x4, y4), 3)

def drawAgents(self):
	for agentIndex in self.agents:
		agent = self.agents[agentIndex]
		self.draw.circle(self.screen, self.AGENT_COLOR, agent.coordinates, agent.radius)

def drawPath(self, path): # TODO fix for nodepath
	x1, y1 = self.G.nodes[self.rooms[self.target].start]['coordinates']
	x2, y2 = self.G.nodes[self.rooms[self.target].end]['coordinates']
	width = self.G.edges[self.rooms[self.target].start, self.rooms[self.target].end]['width']

	sinAcross = self.G.edges[self.rooms[self.target].start, self.rooms[self.target].end]['sinAcross']
	cosAcross = self.G.edges[self.rooms[self.target].start, self.rooms[self.target].end]['cosAcross']
	sinAlong = self.G.edges[self.rooms[self.target].start, self.rooms[self.target].end]['sinAlong']
	cosAlong = self.G.edges[self.rooms[self.target].start, self.rooms[self.target].end]['cosAlong']

	dist = self.rooms[self.target].distance
	x3 = x1 + cosAlong*dist - np.sign(dist)*cosAcross*width/2 + cosAlong*10
	y3 = y1 + sinAlong*dist - np.sign(dist)*sinAcross*width/2 + sinAlong*10

	if len(path) == 0:
		self.draw.line(self.screen, self.PATH_COLOR, (self.robot.x, self.robot.y), (x3, y3), 2)
	else:
		self.draw.line(self.screen, self.PATH_COLOR, (self.robot.x, self.robot.y), path[0], 2)
		self.draw.line(self.screen, self.PATH_COLOR, path[-1], (x3, y3), 2)

		for i in range(1, len(path)):
			self.draw.line(self.screen, self.PATH_COLOR, path[i-1], path[i], 2)

def drawWaypoints(self):
	for point in self.waypointPath:
		self.draw.circle(self.screen, self.WAYPOINT_COLOR, point, 5)

def drawRobot(self):
	self.draw.circle(self.screen, self.ROBOT_COLOR, (self.robot.x, self.robot.y), self.ROBOT_SIZE)
	robotFront = (self.robot.x + np.cos(self.robot.theta)*self.ROBOT_SIZE, self.robot.y + np.sin(self.robot.theta)*self.ROBOT_SIZE)
	self.draw.line(self.screen, self.LANE_COLOR, (self.robot.x, self.robot.y), robotFront, 1)