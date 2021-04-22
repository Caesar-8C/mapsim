import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import numpy as np
import pygame as pyg
import networkx as nx
import time

from agent import Agent
from robot import Robot
from rosBridge import Bridge
from draw import Draw
from utils import tupleSubtract, tupleDistance, Room

class Map:
	from eventHandlerLoop import eventHandlerLoop, mouseScale, mouseDescale
	from draw import drawNodes, drawCorridors, drawLanes, drawRooms, drawAgents, drawPath, drawWaypoints, drawRobot
	from fileManager import loadGraph, saveGraph, clearData
	from pathPlanning import 	computeNodePathDistance, calculateNodePath, calculateWaypointPath,\
								addEdgeWaypoints, addTargetWaypoints, removeWaypointOverlap, getLaneCoordinates,\
								getClosestLane, calculatePathMap
	from futurePrediction import predictFuture, pickCorridors, getActiveAgents, backupAgents, getPreviousDistance,\
								getNextEdge, getCurrentEdge, getDirection

	def __init__(self):
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
		self.WAYPOINT_MARGIN = 2
		self.FUTURE_PREDICTION_TIME = 10
		self.FAST_FORWARD = 10
		self.PREDICTION_MARGIN = 30
		self.RUNNING_VELOCITY_THRESHOLD = 20
		self.ROBOT_INIT_POSE = (200, 300, 0)
		# self.ROBOT_INIT_POSE = (10, 0, 0)
		# self.ROBOT_INIT_POSE = (0, 0, 0)



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

		self.robot = Robot(*self.ROBOT_INIT_POSE, self)
		self.bridge = Bridge(self)
		self.controlAction = [0, 0, 0]

		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

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

			width = self.G.edges[start, end]['width']
			lanes = int(width/self.LANE_PIXEL_WIDTH)
			if lanes == 0:
				lanes = 1
			self.G.edges[start, end]['lanes'] = lanes
			laneWidth = width/lanes
			self.G.edges[start, end]['laneWidth'] = laneWidth

	def add_node(self, name, x, y):
		self.G.add_node(name)
		self.G.nodes[name]['coordinates'] = (x, y)

	def add_edge(self, start, end, width):
		self.G.add_edge(start, end)
		self.G.edges[start, end]['width'] = width
		self.G.edges[start, end]['length'] = tupleDistance(self.G.nodes[start]['coordinates'], self.G.nodes[end]['coordinates'])

		lanes = int(width/self.LANE_PIXEL_WIDTH)
		if lanes == 0:
			lanes = 1
		self.G.edges[start, end]['lanes'] = lanes
		laneWidth = width/lanes
		self.G.edges[start, end]['laneWidth'] = laneWidth


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
		# agent.setTargetLane(1)

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
			sinAcross = self.G.edges[edge]['sinAcross']
			cosAcross = self.G.edges[edge]['cosAcross']

			x3 = x1 + cosAcross*halfWidth
			y3 = y1 + sinAcross*halfWidth

			x4 = x1 - cosAcross*halfWidth
			y4 = y1 - sinAcross*halfWidth

			x6 = x2 + cosAcross*halfWidth
			y6 = y2 + sinAcross*halfWidth

			AM = tupleSubtract(pos, (x3, y3))
			AB = tupleSubtract((x4, y4), (x3, y3))
			AD = tupleSubtract((x6, y6), (x3, y3))

			if 0<np.dot(AM,AB) and np.dot(AM,AB)<np.dot(AB,AB) and 0<np.dot(AM,AD) and np.dot(AM,AD)<np.dot(AD,AD):
				edge_candidates.append(edge)

		return edge_candidates


	def pickNewTarget(self, newTarget=None):
		if not newTarget is None:
			self.target = newTarget
			return

		if len(self.rooms) > 1:
			while True:
				r = np.random.randint(len(self.rooms))
				newTarget = list(self.rooms)[r]
				if self.target != newTarget:
					self.target = newTarget
					break


	def moveAgents(self):
		for agentIndex in list(self.agents):
			self.agents[agentIndex].move()

	def main(self):
		self.run_agent(2, 1)
		self.agents[0].setTargetLane(2)
		while True:
			self.clock.tick(self.fps)

			self.robot.move(self.controlAction)
			if not self.bridge.enabled:
				self.moveAgents()


			self.calculateNodePath()
			predictedEmptyLanes = self.predictFuture()
			self.calculateWaypointPath(predictedEmptyLanes)

			if not self.bridge.enabled and self.robot.automoveEnabled and len(self.waypointPath) > 0:
				self.robot.automove(self.waypointPath[0])
			if self.bridge.enabled and not self.robot.automoveEnabled and len(self.waypointPath) > 0:
				angle = 0
				try:
					if not self.robot.edge is None:
						if len(self.waypointPath) > 1:
							if len(self.nodePath) > 0:
								direction = 1 if self.nodePath[0] == self.G.edges[self.robot.edge]['end'] else -1
							else:
								direction = np.sign(abs(self.rooms[self.target].distance) - self.robot.distance)
							angle = direction*self.G.edges[self.robot.edge]['angleAlong'] + np.pi
						else:
							direction = np.sign(self.rooms[self.target].distance)
							angle = direction*self.G.edges[self.robot.edge]['angleAlong'] + np.pi

					else:
						w = self.waypointPath[0]
						x = self.robot.x
						y = self.robot.y
						angle = np.arctan2(w[1]-y, w[0]-x)

				except:
					angle = 0
				# print('angle: ', angle)
				if self.robot.stopMoving:
					self.bridge.publish((self.robot.x, self.robot.y), angle)
				else:
					# print('goal: ', self.waypointPath[0])
					self.bridge.publish(self.waypointPath[0], angle)

			self.eventHandlerLoop()
			

			self.screen.fill(self.bgcolour)
			self.drawNodes(color=self.CORRIDOR_COLOR)
			self.drawCorridors()
			self.drawNodes(radius=self.NODE_CORE_SIZE)
			self.drawRooms()
			self.drawAgents()
			self.drawPath(self.waypointPath)
			self.drawWaypoints()
			self.drawRobot()

			self.predictFuture()


			pyg.display.flip()
			# print(self.agents[0].lane, ' ', end='')

			# self.chooseTarget()

if __name__ == '__main__':
	map = Map()

	# map.bridge.enable()
	map.robot.enableAutomove()
	map.loadGraph('data/graph2.txt')
	# map.loadGraph('data/graph_test.txt')
	# map.loadGraph('data/map.txt')

	map.main()