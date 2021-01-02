import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import numpy as np
import pygame as pyg
import networkx as nx
import time

from utils import *
from draw import Draw

class Map:
	from eventHandlerLoop import eventHandlerLoop, mouseScale, mouseDescale
	from draw import drawNodes, drawCorridors, drawLanes, drawRooms, drawAgents, drawPath, drawWaypoints, drawRobot
	from fileManager import loadGraph, saveGraph, clearData
	from pathPlanning import 	computeNodePathDistance, calculateNodePath, calculateWaypointPath,\
								addEdgeWaypoints, addTargetWaypoints, removeWaypointOverlap, getLaneCoordinates,\
								getClosestLane, calculatePathMap
	from futurePrediction import predictFuture, pickCorridors, removeAgents

	def __init__(self):
		self.G = nx.Graph()
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
		self.pathMap = None
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
		self.WAYPOINT_COLOR = (200, 100, 100)
		self.WAYPOINT_DISTANCE = 10
		self.FUTURE_PREDICTION_TIME = 3

		self.AGENT1 = pyg.USEREVENT+1
		self.AGENT2 = pyg.USEREVENT+2
		self.AGENT3 = pyg.USEREVENT+3
		# pyg.time.set_timer(self.AGENT1, 5000)
		# pyg.time.set_timer(self.AGENT2, 1200)
		# pyg.time.set_timer(self.AGENT3, 4000)

	def mapChanged(self):
		self.recalculateEdgeParams()
		self.calculatePathMap()

	def recalculateEdgeParams(self):
		for edge in self.G.edges:
			self.G.edges[edge]['length'] = tupleDistance(self.G.nodes[edge[0]]['coordinates'], self.G.nodes[edge[1]]['coordinates'])
			start = self.G.edges[edge]['start']
			end = self.G.edges[edge]['end']

			x1, y1 = self.G.nodes[start]['coordinates']
			x2, y2 = self.G.nodes[end]['coordinates']
			angleAcross = np.arctan2(x1-x2, y2-y1)
			angleAlong = np.arctan2(y1-y2, x1-x2)

			self.G.edges[start, end]['angleAcross'] = angleAcross
			self.G.edges[start, end]['angleAlong'] = angleAlong
			self.G.edges[start, end]['sinAcross'] = np.sin(angleAcross)
			self.G.edges[start, end]['cosAcross'] = np.cos(angleAcross)
			self.G.edges[start, end]['sinAlong'] = np.sin(angleAlong)
			self.G.edges[start, end]['cosAlong'] = np.cos(angleAlong)

	def add_node(self, name, x, y):
		self.G.add_node(name)
		self.G.nodes[name]['coordinates'] = (x, y)

	def add_edge(self, start, end, width):
		self.G.add_edge(start, end)
		self.G.edges[start, end]['width'] = width
		self.G.edges[start, end]['lanes'] = 1
		self.G.edges[start, end]['laneWidth'] = width
		self.G.edges[start, end]['length'] = tupleDistance(self.G.nodes[start]['coordinates'], self.G.nodes[end]['coordinates'])

		x1, y1 = self.G.nodes[start]['coordinates']
		x2, y2 = self.G.nodes[end]['coordinates']
		angleAcross = np.arctan2(x1-x2, y2-y1)
		angleAlong = np.arctan2(y1-y2, x1-x2)

		self.G.edges[start, end]['angleAcross'] = angleAcross
		self.G.edges[start, end]['angleAlong'] = angleAlong
		self.G.edges[start, end]['sinAcross'] = np.sin(angleAcross)
		self.G.edges[start, end]['cosAcross'] = np.cos(angleAcross)
		self.G.edges[start, end]['sinAlong'] = np.sin(angleAlong)
		self.G.edges[start, end]['cosAlong'] = np.cos(angleAlong)

		self.G.edges[start, end]['start'] = start
		self.G.edges[start, end]['end'] = end

	def add_room(self, number, start, end, distance):
		# TODO check if edge exists
		room = Room(start, end, distance)
		self.rooms[number] = room

	def run_agent(self, origin_node=None, end_node=None, lane=None, speed=None):
		agent = Agent(self, self.agentNameCounter, origin_node, end_node, lane, speed)
		self.agents[self.agentNameCounter] = agent
		self.agentNameCounter += 1
		agent.setTargetLane(2)

	def activeReset(self):
		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

	def activateElement(self, pos):
		node = self.insideNodeCheck(pos, self.NODE_CORE_SIZE)
		if node != None:
			self.activeNode = node
			return

		edge_candidates = self.insideEdgeCheck(pos)
		if len(edge_candidates) > 0:
			self.activeEdge = edge_candidates[0]
			return

		self.activeBackground = True

	def insideNodeCheck(self, pos, size):
		for node in self.G.nodes:
			dist = tupleDistance(self.G.nodes[node]['coordinates'], pos)
			if dist < size:
				return node
		return None

	def insideEdgeCheck(self, pos):
		edge_candidates = []

		for edge in self.G.edges:
			x1, y1 = self.G.nodes[edge[0]]['coordinates']
			x2, y2 = self.G.nodes[edge[1]]['coordinates']
			halfWidth = self.G.edges[edge[0], edge[1]]['width']/2

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
				edge_candidates.append(edge)

		return edge_candidates


	def moveAgents(self):
		for agentIndex in list(self.agents):
			self.agents[agentIndex].move()

	def main(self):
		self.calculatePathMap()
		while True:
			self.clock.tick(self.fps)

			self.robot.move(self.controlAction)
			self.moveAgents()


			self.calculateNodePath()
			self.predictFuture()
			self.calculateWaypointPath()
			self.eventHandlerLoop()
			

			self.screen.fill(self.bgcolour)
			self.drawNodes(color=self.CORRIDOR_COLOR)
			self.drawCorridors()
			self.drawNodes(radius=self.NODE_CORE_SIZE)
			self.drawRooms()
			self.drawAgents()
			self.drawPath(self.waypointPath)
			# self.drawWaypoints()
			self.drawRobot()


			pyg.display.flip()

if __name__ == '__main__':
	map = Map()

	map.loadGraph('data/graph2.txt')

	map.main()