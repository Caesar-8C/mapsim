import numpy as np
import pygame as pyg
import networkx as nx
import time

from utils import *
from draw import Draw

class Map:
	from eventHandlerLoop import eventHandlerLoop, mouseScale, mouseDescale
	from draw import drawNodes, drawCorridors, drawLanes, drawRooms, drawAgents
	from fileManager import loadGraph, saveGraph

	def __init__(self):
		self.G = nx.DiGraph()
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

		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

		self.NODE_SIZE = 10
		self.MIN_CLICKABLE_CORRIDOR_WIDTH = 5
		self.LANE_PIXEL_WIDTH = 20
		self.NODE_COLOR = (0, 128, 0)
		self.CORRIDOR_COLOR = (255, 255, 255)
		self.LANE_COLOR = (0, 100, 0)
		self.AGENT_COLOR = (0, 0, 100)
		self.ROOM_COLOR = (255, 0, 0)

		self.AGENT1 = pyg.USEREVENT+1
		self.AGENT2 = pyg.USEREVENT+2
		self.AGENT3 = pyg.USEREVENT+3
		pyg.time.set_timer(self.AGENT1, 1000)
		pyg.time.set_timer(self.AGENT2, 1200)
		pyg.time.set_timer(self.AGENT3, 4000)

	def add_node(self, name, x, y):
		self.G.add_node(name)
		self.G.nodes[name]['coordinates'] = (x, y)

	def add_edge(self, start, end, width):
		self.G.add_edge(start, end)
		self.G.edges[start, end]['width'] = width
		self.G.edges[start, end]['lanes'] = 1
		self.G.edges[start, end]['laneWidth'] = width

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
			dist = np.linalg.norm(tupleSubtract(self.G.nodes[node]['coordinates'], pos))
			if dist < self.NODE_SIZE:
				self.activeNode = node
				return

		for edge in self.G.edges:
			x0, y0 = pos
			x1, y1 = self.G.nodes[edge[0]]['coordinates']
			x2, y2 = self.G.nodes[edge[1]]['coordinates']
			if x2-x1 == 0 or y1-y2 == 0: # TODO fix axis parallel lines
				continue
			a = 1./(x2 - x1)
			b = 1./(y1 - y2)
			c = float(y1)/(y2-y1) - float(x1)/(x2-x1)
			x = int((b*(b*x0 - a*y0)-a*c)/(a**2 + b**2))
			y = int((a*(a*y0 - b*x0)-b*c)/(a**2 + b**2))
			if between(x, x1, x2) and between(y, y1, y2):
				dist = np.abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/np.sqrt((y2-y1)**2 + (x2-x1)**2)
				if dist < self.G.edges[edge]['width']/2 or dist < self.MIN_CLICKABLE_CORRIDOR_WIDTH:
					self.activeEdge = edge
					return
		self.activeBackground = True

	def inside(self, pos):
		corridor = (1, 2)
		x1, y1 = self.G.nodes[corridor[0]]['coordinates']
		x2, y2 = self.G.nodes[corridor[1]]['coordinates']
		halfWidth = self.G.edges[corridor[0], corridor[1]]['width']/2

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

		if (0<self.dot(AM,AB) and self.dot(AM,AB)<self.dot(AB,AB)) or (0<self.dot(AM,AD) and self.dot(AM,AD)<self.dot(AD,AD)):
			return True
		return False

	def dot(self, a, b):
		return a[0]*b[0] + a[1]*b[1]

	def main(self):
		while True:
			self.screen.fill(self.bgcolour)
			self.clock.tick(self.fps)
			self.eventHandlerLoop()

			self.drawCorridors()
			self.drawNodes()
			self.drawRooms()
			self.drawAgents()

			pyg.display.flip()

if __name__ == '__main__':
	map = Map()

	map.loadGraph('data/graph2.txt')

	map.main()
