import numpy as np
import networkx as nx
from utils import *

def computeNodePathDistance(self, path):
	sum = 0
	for i in range(1, len(path)):
		sum += self.G.edges[path[i-1], path[i]]['length']
	return sum

def calculatePathMap(self):
	self.pathMap = nx.shortest_path(self.G, weight='length', method='dijkstra')

def calculateNodePath(self):
	targetEdge = (self.rooms[self.target].start, self.rooms[self.target].end)
	self.nodePath = []

	if self.robot.node != None:
		paths = []
		paths.append(self.pathMap[self.robot.node][targetEdge[0]].copy())
		paths.append(self.pathMap[self.robot.node][targetEdge[1]].copy())

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
					break
		else:
			for edge in self.robot.edge_candidates:
				if edge == targetEdge:
					self.robot.node = None
					self.robot.setEdge(edge)
					del self.nodePath[0]

	elif self.robot.edge != None:
		if self.robot.edge != targetEdge:
			paths = []
			paths.append(self.pathMap[self.robot.edge[0]][targetEdge[0]].copy())
			paths.append(self.pathMap[self.robot.edge[0]][targetEdge[1]].copy())
			paths.append(self.pathMap[self.robot.edge[1]][targetEdge[0]].copy())
			paths.append(self.pathMap[self.robot.edge[1]][targetEdge[1]].copy())

			dists = []
			dists.append(int(self.computeNodePathDistance(paths[0]) + tupleDistance(self.G.nodes[self.robot.edge[0]]['coordinates'], (self.robot.x, self.robot.y))) + abs(self.rooms[self.target].distance))
			dists.append(int(self.computeNodePathDistance(paths[1]) + tupleDistance(self.G.nodes[self.robot.edge[0]]['coordinates'], (self.robot.x, self.robot.y))) + self.G.edges[targetEdge]['length'] - abs(self.rooms[self.target].distance))
			dists.append(int(self.computeNodePathDistance(paths[2]) + tupleDistance(self.G.nodes[self.robot.edge[1]]['coordinates'], (self.robot.x, self.robot.y))) + abs(self.rooms[self.target].distance))
			dists.append(int(self.computeNodePathDistance(paths[3]) + tupleDistance(self.G.nodes[self.robot.edge[1]]['coordinates'], (self.robot.x, self.robot.y))) + self.G.edges[targetEdge]['length'] - abs(self.rooms[self.target].distance))

			self.nodePath = paths[np.argmin(dists)]


def calculateWaypointPath(self, predictedBusyLanes):
	self.waypointPath = []
	targetEdge = (self.rooms[self.target].start, self.rooms[self.target].end)

	if self.robot.edge == targetEdge:
		self.addTargetWaypoints(predictedBusyLanes)
		return

	if self.robot.edge != None:
		self.addEdgeWaypoints(predictedBusyLanes, self.robot.edge, self.nodePath[0])

	for i in range(len(self.nodePath)-1):
		for edge in self.G.edges:
			if self.nodePath[i] in edge and self.nodePath[i+1] in edge:
				index = len(self.waypointPath)-1
				self.addEdgeWaypoints(predictedBusyLanes, edge, self.nodePath[i+1])
				self.removeWaypointOverlap(index)
				break
	
	if len(self.nodePath) != 0:
		index = len(self.waypointPath)-2 #TODO
		self.addTargetWaypoints(predictedBusyLanes, self.nodePath[-1])
		self.removeWaypointOverlap(index)


def addEdgeWaypoints(self, predictedBusyLanes, edge, node):
	if edge == self.robot.edge:
		direction = 1 if self.G.edges[edge]['end'] == node else -1
		maxDist = self.G.edges[edge]['length']
		distance =  0 if direction == 1 else maxDist

		if self.robot.distance is None:
			targetDist = 0 if direction == 1 else maxDist
		else:
			targetDist = self.robot.distance

		while direction*distance < direction*(targetDist):
			distance += direction*self.WAYPOINT_DISTANCE
	else:
		if edge[0] == node:
			distance = self.G.edges[edge]['length'] - self.WAYPOINT_DISTANCE
			direction = -1
		else:
			distance = self.WAYPOINT_DISTANCE
			direction = 1


	maxDist = self.G.edges[edge]['length']
	lanes = list(range(self.G.edges[edge]['lanes']))

	velocity = self.robot.getVelocity() if self.bridge.enabled else self.robot.maxVelocity[0]
	timeStep = velocity*self.FAST_FORWARD
	timeStepDist = distance if direction == 1 else maxDist - distance

	while 0 <= distance and distance <= maxDist:
		if len(predictedBusyLanes) > 0:
			emptyLanes = subtractLists(lanes, predictedBusyLanes[0])
		else:
			emptyLanes = lanes

		self.robot.stopMoving = True if len(emptyLanes) == 0 else False

		if len(emptyLanes) == 0:
			lane = 0
		elif direction == 1:
			lane = np.max(emptyLanes)
		else:
			lane = np.min(emptyLanes)


		if 1 in emptyLanes:
			lane = 1

		self.waypointPath.append(self.getLaneCoordinates(edge, distance, lane))
		distance += direction*self.WAYPOINT_DISTANCE
		timeStepDist += self.WAYPOINT_DISTANCE

		if timeStepDist > timeStep and len(predictedBusyLanes) > 0:
			timeStepDist -= timeStep
			del predictedBusyLanes[0]


def addTargetWaypoints(self, predictedBusyLanes, node = None):
	edge = (self.rooms[self.target].start, self.rooms[self.target].end)
	roomDistance = abs(self.rooms[self.target].distance)
	targetLane = 0 if self.rooms[self.target].distance < 0 else self.G.edges[edge]['lanes']-1
	maxDist = self.G.edges[edge]['length']

	if edge == self.robot.edge:
		direction = np.sign(roomDistance - self.robot.distance)
		distance =  0 if direction == 1 else maxDist

		if self.robot.distance is None:
			targetDist = 0 if direction == 1 else maxDist
		else:
			targetDist = self.robot.distance

		while direction*distance < direction*(targetDist - direction*self.WAYPOINT_DISTANCE):
			distance += direction*self.WAYPOINT_DISTANCE
	else:
		if self.G.edges[edge]['start'] == node:
			distance = 0
			direction = 1
		else:
			distance = maxDist
			direction = -1

	lanes = list(range(self.G.edges[edge]['lanes']))


	velocity = self.robot.getVelocity() if self.bridge.enabled else self.robot.maxVelocity[0]
	timeStep = velocity*self.FAST_FORWARD
	timeStepDist = distance if direction == 1 else maxDist - distance


	while distance <= roomDistance-self.WAYPOINT_DISTANCE or roomDistance+self.WAYPOINT_DISTANCE <= distance:
		if len(predictedBusyLanes) > 0:
			emptyLanes = subtractLists(lanes, predictedBusyLanes[0])
		else:
			emptyLanes = lanes

		self.robot.stopMoving = True if len(emptyLanes) == 0 else False

		if len(emptyLanes) == 0:
			lane = 0
		elif direction == 1:
			lane = np.max(emptyLanes)
		else:
			lane = np.min(emptyLanes)

		lane = 1

		self.waypointPath.append(self.getLaneCoordinates(edge, distance, lane))
		distance += direction*self.WAYPOINT_DISTANCE
		timeStepDist += self.WAYPOINT_DISTANCE

		if timeStepDist > timeStep and len(predictedBusyLanes) > 0:
			timeStepDist -= timeStep
			del predictedBusyLanes[0]

	self.waypointPath.append(self.getLaneCoordinates(edge, distance, targetLane))

	while len(self.waypointPath) > 1 and tupleDistance((self.robot.x, self.robot.y), self.waypointPath[0]) < self.WAYPOINT_MARGIN*self.WAYPOINT_DISTANCE:
		del self.waypointPath[0]

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

	start = self.G.edges[edge]['start']
	end = self.G.edges[edge]['end']
	x1, y1 = self.G.nodes[start]['coordinates']
	x2, y2 = self.G.nodes[end]['coordinates']
	halfWidth = self.G.edges[edge]['width']/2

	sinAcross = self.G.edges[edge]['sinAcross']
	cosAcross = self.G.edges[edge]['cosAcross']
	sinAlong = self.G.edges[edge]['sinAlong']
	cosAlong = self.G.edges[edge]['cosAlong']

	distanceAcross = (laneIndex + 0.5) * self.G.edges[edge]['laneWidth'] - halfWidth

	x3 = x1 - cosAcross*distanceAcross - cosAlong*distanceAlong
	y3 = y1 - sinAcross*distanceAcross - sinAlong*distanceAlong

	return (x3, y3)