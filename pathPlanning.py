import numpy as np
import networkx as nx
from utils import *

def computeNodePathDistance(self, path):
	sum = 0
	for i in range(1, len(path)):
		sum += self.G.edges[path[i-1], path[i]]['length']
	return sum

def calculateNodePath(self):
	pathMap = nx.shortest_path(self.G, weight='length', method='dijkstra')
	targetEdge = (self.rooms[self.target].start, self.rooms[self.target].end)
	self.nodePath = []

	if self.robot.node != None:
		paths = []
		paths.append(pathMap[self.robot.node][targetEdge[0]])
		paths.append(pathMap[self.robot.node][targetEdge[1]])

		dists = []
		dists.append(int(self.computeNodePathDistance(paths[0])) + abs(self.rooms[self.target].distance))
		dists.append(int(self.computeNodePathDistance(paths[1])) + self.G.edges[targetEdge]['length'] - abs(self.rooms[self.target].distance))

		self.nodePath = paths[np.argmin(dists)]

		if len(self.nodePath) > 1: # TODO simplify?
			for edge in self.robot.edge_candidates:
				if self.nodePath[1] in edge:
					self.robot.node = None
					self.robot.setEdge(edge)
					del self.nodePath[0]
		else:
			for edge in self.robot.edge_candidates:
				if edge == targetEdge:
					self.robot.node = None
					self.robot.setEdge(edge)
					del self.nodePath[0]

	elif self.robot.edge != None:
		if self.robot.edge != targetEdge:
			paths = []
			paths.append(pathMap[self.robot.edge[0]][targetEdge[0]])
			paths.append(pathMap[self.robot.edge[0]][targetEdge[1]])
			paths.append(pathMap[self.robot.edge[1]][targetEdge[0]])
			paths.append(pathMap[self.robot.edge[1]][targetEdge[1]])

			dists = []
			dists.append(int(self.computeNodePathDistance(paths[0]) + tupleDistance(self.G.nodes[self.robot.edge[0]]['coordinates'], (self.robot.x, self.robot.y))) + abs(self.rooms[self.target].distance))
			dists.append(int(self.computeNodePathDistance(paths[1]) + tupleDistance(self.G.nodes[self.robot.edge[0]]['coordinates'], (self.robot.x, self.robot.y))) + self.G.edges[targetEdge]['length'] - abs(self.rooms[self.target].distance))
			dists.append(int(self.computeNodePathDistance(paths[2]) + tupleDistance(self.G.nodes[self.robot.edge[1]]['coordinates'], (self.robot.x, self.robot.y))) + abs(self.rooms[self.target].distance))
			dists.append(int(self.computeNodePathDistance(paths[3]) + tupleDistance(self.G.nodes[self.robot.edge[1]]['coordinates'], (self.robot.x, self.robot.y))) + self.G.edges[targetEdge]['length'] - abs(self.rooms[self.target].distance))

			self.nodePath = paths[np.argmin(dists)]


def calculateWaypointPath(self):
	self.waypointPath = []
	targetEdge = (self.rooms[self.target].start, self.rooms[self.target].end)

	if self.robot.edge == targetEdge:
		self.addTargetWaypoints()
		return

	if self.robot.edge != None:
		self.addEdgeWaypoints(self.robot.edge, self.nodePath[0])

	for i in range(len(self.nodePath)-1):
		for edge in self.G.edges:
			if self.nodePath[i] in edge and self.nodePath[i+1] in edge:
				self.addEdgeWaypoints(edge, self.nodePath[i+1])
				self.removeWaypointOverlap()
				break
	
	self.addTargetWaypoints(self.nodePath[-1])
	self.removeWaypointOverlap()


def addEdgeWaypoints(self, edge, node):
	if edge == self.robot.edge:
		distance = self.robot.distance
		direction = 1 if edge[1] == node else -1
		lane = self.getClosestLane(edge, distance)
	else:
		if edge[0] == node:
			distance = self.G.edges[edge]['length']
			direction = -1
		else:
			distance = 0
			direction = 1
		lane = 0 # fix (pick closest lane to a robot)


	maxDist = self.G.edges[edge]['length']

	while 0 <= distance and distance <= maxDist:
		# TODO make a function here to determine the best lane
		self.waypointPath.append(self.getLaneCoordinates(edge, distance, lane))
		distance += direction*self.WAYPOINT_DISTANCE


def addTargetWaypoints(self, node = None):
	edge = (self.rooms[self.target].start, self.rooms[self.target].end)
	targetDistance = abs(self.rooms[self.target].distance)
	corridorSide = True if self.rooms[self.target].distance < 0 else False
	targetLane = 0 if self.rooms[self.target].distance > 0 else self.G.edges[edge]['lanes']-1

	if edge == self.robot.edge:
		distance = self.robot.distance
		direction = 1 if distance < targetDistance else -1
	else:
		if edge[1] == node:
			distance = self.G.edges[edge]['length']
			direction = -1
		else:
			distance = 0
			direction = 1

	while True:
		# TODO make a function here to determine the best lane
		self.waypointPath.append(self.getLaneCoordinates(edge, distance, targetLane))
		distance += direction*self.WAYPOINT_DISTANCE
		if np.sign(direction) == np.sign(distance - targetDistance):
			break

def removeWaypointOverlap(self):
	mindist = self.WAYPOINT_DISTANCE
	index1 = 0
	index2 = 0
	for i1, point1 in enumerate(self.waypointPath):
		for i2, point2 in enumerate(self.waypointPath):
			if i1 == i2:
				continue
			dist = tupleDistance(point1, point2)
			if dist < mindist:
				mindist = dist
				index1 = i1
				index2 = i2

	if index1 < index2:
		del self.waypointPath[index1+1:index2]


def calculateWaypointPath_backup(self):
	pos = (self.robot.x, self.robot.y)
	self.waypointPath = []
	direction = 1
	if self.robot.node != None:
		if len(self.nodePath) < 2:
			edge = (self.rooms[self.target].start, self.rooms[self.target].end) # [end, start] distance - targetDistance
		else:
			edge = (self.nodePath[0], self.nodePath[1]) # [1, 0]
		distance = 0
	elif self.robot.edge != None:
		edge = self.robot.edge
		distance = self.robot.distance
		if len(self.nodePath) > 0:
			lane = self.getClosestLane(edge, distance)
			if self.robot.edge[0] == self.nodePath[0]:
				direction = -1
		else:
			if self.rooms[self.target].distance < 0:
				lane = self.G.edges[edge]['lanes'] - 1
			else:
				lane = 0
			if distance > abs(self.rooms[self.target].distance):
				direction = -1



	maxDist = self.G.edges[edge]['length']

	while 0 < distance and distance < maxDist:
		self.waypointPath.append(self.getLaneCoordinates(edge, distance, lane))
		distance += direction*self.WAYPOINT_DISTANCE

		if edge == (self.rooms[self.target].start, self.rooms[self.target].end):
			if direction > 0 and distance > abs(self.rooms[self.target].distance):
				break
			elif direction < 0 and distance < abs(self.rooms[self.target].distance):
				break


def getClosestLane(self, edge=None, distance=None, pos=None):
	if edge == None:
		edge = self.robot.edge
	if distance == None:
		distance = self.robot.distance
	if pos == None:
		pos = (self.robot.x, self.robot.y)

	laneNum = self.G.edges[edge]['lanes']
	laneDistances = []

	for i in range(laneNum):
		coordinates = self.getLaneCoordinates(edge, distance, i)
		laneDistances.append(tupleDistance(pos, coordinates))

	return np.argmin(laneDistances)


def getLaneCoordinates(self, edge, distanceAlong=0, laneIndex=1):
	if edge == None:
		return None

	if laneIndex > self.G.edges[edge]['lanes']:
		return None
	if laneIndex < 0:
		pass

	x1, y1 = self.G.nodes[edge[0]]['coordinates']
	x2, y2 = self.G.nodes[edge[1]]['coordinates']
	halfWidth = self.G.edges[edge[0], edge[1]]['width']/2

	angleAcross = np.arctan2(x1-x2, y2-y1)
	angleAlong = np.arctan2(y1-y2, x1-x2)

	distanceAcross = (laneIndex + 0.5) * self.G.edges[edge]['laneWidth'] - halfWidth

	x3 = x1 + np.cos(angleAcross)*distanceAcross - np.cos(angleAlong)*distanceAlong
	y3 = y1 + np.sin(angleAcross)*distanceAcross - np.sin(angleAlong)*distanceAlong

	return (x3, y3)