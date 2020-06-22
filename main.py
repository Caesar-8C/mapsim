import numpy as np
import pygame as pyg
import networkx as nx
import time

from utils import *
from draw import Draw

class Map:
	from eventHandlerLoop import eventHandlerLoop, mouseScale, mouseDescale
	from draw import drawNodes, drawCorridors, drawLanes, drawRooms, drawAgents, drawNodePath, drawRobot
	from fileManager import loadGraph, saveGraph

	def __init__(self):
		self.G = nx.Graph() # removed Di, TODO test
		self.bgcolour = 0x2F, 0x4F, 0x4F
		self.size = self.width, self.height = 800, 600
		pyg.init()
		self.screen = pyg.display.set_mode(self.size)
		self.clock = pyg.time.Clock()
		self.rooms = {}
		self.agents = {}
		self.fps = 60
		self.agentNameCounter = 0
		self.draw = Draw()
		self.rel_residual = (0, 0)
		self.nodePath = []
		self.waypointPath = []
		self.target = 501

		self.robot = Robot(200, 300, self)
		self.controlAction = [0, 0, 0]

		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

		self.NODE_SIZE = 30
		self.NODE_CORE_SIZE = 10
		self.MIN_CLICKABLE_CORRIDOR_WIDTH = 5
		self.LANE_PIXEL_WIDTH = 20
		self.ROBOT_SIZE = 10
		self.ROBOT_COLOR = (255, 0, 0)
		self.NODE_COLOR = (0, 128, 0)
		self.CORRIDOR_COLOR = (255, 255, 255)
		self.LANE_COLOR = (0, 100, 0)
		self.AGENT_COLOR = (0, 0, 100)
		self.ROOM_COLOR = (255, 0, 0)
		self.PATH_COLOR = (128, 0, 128)
		self.WAYPOINT_DISTANCE = 10

		self.AGENT1 = pyg.USEREVENT+1
		self.AGENT2 = pyg.USEREVENT+2
		self.AGENT3 = pyg.USEREVENT+3
		# pyg.time.set_timer(self.AGENT1, 1000)
		# pyg.time.set_timer(self.AGENT2, 1200)
		# pyg.time.set_timer(self.AGENT3, 4000)

	def recalculateEdgeLengths(self):
		for edge in self.G.edges:
			self.G.edges[edge]['length'] = tupleDistance(self.G.nodes[edge[0]]['coordinates'], self.G.nodes[edge[1]]['coordinates'])

	def add_node(self, name, x, y):
		self.G.add_node(name)
		self.G.nodes[name]['coordinates'] = (x, y)

	def add_edge(self, start, end, width):
		self.G.add_edge(start, end)
		self.G.edges[start, end]['width'] = width
		self.G.edges[start, end]['lanes'] = 1
		self.G.edges[start, end]['laneWidth'] = width
		self.G.edges[start, end]['length'] = tupleDistance(self.G.nodes[start]['coordinates'], self.G.nodes[end]['coordinates'])

	def add_room(self, number, start, end, distance):
		# TODO check if edge exists
		room = Room(start, end, distance)
		self.rooms[number] = room

	def run_agent(self, origin_node, end_node, lane, speed):
		agent = Agent(origin_node, end_node, lane, speed)
		self.agents[self.agentNameCounter] = agent
		self.agentNameCounter += 1

	def activeReset(self):
		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

	def activateElement(self, pos):
		for node in self.G.nodes:
			dist = tupleDistance(self.G.nodes[node]['coordinates'], pos)
			if dist < self.NODE_CORE_SIZE:
				self.activeNode = node
				return

		for edge in self.G.edges:
			x1, y1 = self.G.nodes[edge[0]]['coordinates']
			x2, y2 = self.G.nodes[edge[1]]['coordinates']
			halfWidth = max(self.G.edges[edge[0], edge[1]]['width']/2, self.MIN_CLICKABLE_CORRIDOR_WIDTH/2)

			angle = np.arctan2(x1-x2, y2-y1)
			x3 = x1 + np.cos(angle)*halfWidth
			y3 = y1 + np.sin(angle)*halfWidth

			x4 = x1 - np.cos(angle)*halfWidth
			y4 = y1 - np.sin(angle)*halfWidth

			x6 = x2 + np.cos(angle)*halfWidth
			y6 = y2 + np.sin(angle)*halfWidth

			AM = tupleSubtract(pos, (x3, y3))
			AB = tupleSubtract((x4, y4), (x3, y3))
			AD = tupleSubtract((x6, y6), (x3, y3))

			if 0<np.dot(AM,AB) and np.dot(AM,AB)<np.dot(AB,AB) and 0<np.dot(AM,AD) and np.dot(AM,AD)<np.dot(AD,AD):
				self.activeEdge = edge
				return

		self.activeBackground = True

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
		for node in self.nodePath:
			self.waypointPath.append(self.G.nodes[node]['coordinates'])

		# for node in self.nodePath:
		# 	dist = tupleDistance(self.G.nodes[node]['coordinates'], self.waypointPath[-1])
		# 	if dist > self.WAYPOINT_DISTANCE:


	def main(self):
		while True:
			self.clock.tick(self.fps)
			
			self.robot.move(self.controlAction)
			self.calculateNodePath()
			# self.calculateWaypointPath()
			self.eventHandlerLoop()
			

			self.screen.fill(self.bgcolour)
			self.drawNodes(color=self.CORRIDOR_COLOR)
			self.drawCorridors()
			self.drawNodes(radius=self.NODE_CORE_SIZE)
			self.drawRooms()
			self.drawAgents()
			self.drawNodePath()
			self.drawRobot()


			pyg.display.flip()

if __name__ == '__main__':
	map = Map()

	map.loadGraph('data/graph2.txt')

	map.main()