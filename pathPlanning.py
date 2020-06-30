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

		if len(self.nodePath) > 1:
			for edge in self.robot.edge_candidates:
				if self.nodePath[1] in edge:
					self.robot.node = None
					self.robot.edge = edge
					del self.nodePath[0]
		else:
			for edge in self.robot.edge_candidates:
				if edge == targetEdge:
					self.robot.node = None
					self.robot.edge = edge
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
	pos = (self.robot.x, self.robot.y)
	self.waypointPath = []
	edge = self.robot.edge # TODO if self.node != None
	node = self.nodePath[0]



	# determine direction of node
	# pick a lane
	# add waypoint
	self.waypointPath.append(self.G.nodes[node]['coordinates'])

	# for node in self.nodePath:
	# 	dist = tupleDistance(self.G.nodes[node]['coordinates'], self.waypointPath[-1])
	# 	if dist > self.WAYPOINT_DISTANCE:


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
	# distanceAlong = self.G.edges[edge]['length']/2 # for testing purposes

	x3 = x1 + np.cos(angleAcross)*distanceAcross - np.cos(angleAlong)*distanceAlong
	y3 = y1 + np.sin(angleAcross)*distanceAcross - np.sin(angleAlong)*distanceAlong

	return (x3, y3)