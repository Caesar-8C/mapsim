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
				index = len(self.waypointPath)-1
				self.addEdgeWaypoints(edge, self.nodePath[i+1])
				self.removeWaypointOverlap(index)
				break
	
	index = len(self.waypointPath)-1
	self.addTargetWaypoints(self.nodePath[-1])
	self.removeWaypointOverlap(index)


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

	self.waypointPath.append(self.getLaneCoordinates(edge, distance, targetLane))
	while True:
		# TODO make a function here to determine the best lane
		distance += direction*self.WAYPOINT_DISTANCE
		if np.sign(direction) == np.sign(distance - targetDistance):
			break
		self.waypointPath.append(self.getLaneCoordinates(edge, distance, targetLane))

def removeWaypointOverlap(self, index):
	maxIndex = len(self.waypointPath)-1

	index1 = index
	index2 = index + 1
	flag = False

	dist = tupleDistance(self.waypointPath[index1], self.waypointPath[index2])

	if index1 > 0:
		dist2 = tupleDistance(self.waypointPath[index1-1], self.waypointPath[index2])

		while dist2 < dist and index1 > 0:
			flag = True
			dist = dist2
			index1 -= 1
			dist2 = tupleDistance(self.waypointPath[index1], self.waypointPath[index2])

	if index2 < maxIndex:
		index2 += 1
		dist2 = tupleDistance(self.waypointPath[index1], self.waypointPath[index2])

		while dist2 < dist and index2 < maxIndex:
			flag = True
			dist = dist2
			index2 += 1
			dist2 = tupleDistance(self.waypointPath[index1], self.waypointPath[index2])

	if flag:
		del self.waypointPath[index1:index2]


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