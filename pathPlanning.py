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

	if self.robot.node != None:
		paths = []
		paths.append(pathMap[self.robot.node][targetEdge[0]])
		paths.append(pathMap[self.robot.node][targetEdge[1]])

		dists = []
		dists.append(int(self.computeNodePathDistance(paths[0])) + abs(self.rooms[self.target].distance))
		dists.append(int(self.computeNodePathDistance(paths[1])) + self.G.edges[targetEdge]['length'] - abs(self.rooms[self.target].distance))

		self.nodePath = paths[np.argmin(dists)]

		for edge in self.robot.edge_candidates:
			if self.nodePath[1] in edge:
				self.robot.edge = edge
				del self.nodePath[0]


	elif self.robot.edge != None:

		if self.robot.edge == targetEdge:
			self.nodePath = []
		else:
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
	start = (self.robot.x, self.robot.y)
	self.waypointPath = [start]
	edge = self.robot.edge
	for node in self.nodePath:
		# determine direction of node
		# pick a lane
		# add waypoint
		self.waypointPath.append(self.G.nodes[node]['coordinates'])

	# for node in self.nodePath:
	# 	dist = tupleDistance(self.G.nodes[node]['coordinates'], self.waypointPath[-1])
	# 	if dist > self.WAYPOINT_DISTANCE: