import numpy as np
import pygame as pyg
import networkx as nx
import time

from utils import *
from draw import Draw

class Map:
	from eventHandlerLoop import eventHandlerLoop, mouseScale, mouseDescale
	from draw import drawCorridor, drawRooms, drawLanes, drawAgents, drawSuccessors
	from fileManager import loadGraph, saveGraph

	def __init__(self):
		self.G = nx.DiGraph()
		self.bgcolour = 0x2F, 0x4F, 0x4F
		self.size = self.width, self.height = 800, 600
		pyg.init()
		self.screen = pyg.display.set_mode(self.size)
		self.clock = pyg.time.Clock()
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

	def run_agent(self, origin_node, end_node, lane, speed):
		agent = Agent(origin_node, end_node, lane, speed)
		self.agents[self.agentNameCounter] = agent
		self.agentNameCounter += 1

	def add_node(self, name, x, y):
		self.G.add_node(name)
		self.G.nodes[name]['coordinates'] = (x, y)

	def add_edge(self, start, end, width):
		self.G.add_edge(start, end)
		self.G.edges[start, end]['width'] = width
		self.G.edges[start, end]['lanes'] = 1
		self.G.edges[start, end]['laneWidth'] = width
		self.G.edges[start, end]['rooms'] = {}

	def activeReset(self):
		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

	def activateElement(self, pos):
		for node in list(self.G.nodes):
			dist = np.linalg.norm(tupleSubtract(self.G.nodes[node]['coordinates'], pos))
			if dist < self.NODE_SIZE:
				self.activeNode = node
				return

		for edge in list(self.G.edges):
			x0, y0 = pos
			x1, y1 = self.G.nodes[edge[0]]['coordinates']
			x2, y2 = self.G.nodes[edge[1]]['coordinates']
			if x2-x1 == 0 or y1-y2 == 0:
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

	def main(self):
		while True:
			self.screen.fill(self.bgcolour)
			self.clock.tick(self.fps)
			self.eventHandlerLoop()

		
			node = 1
			self.drawSuccessors(node, [])
			self.drawAgents()

			pyg.display.flip()

if __name__ == '__main__':
	map = Map()

	map.loadGraph('data/graph2.txt')
	map.G.edges[2, 3]['rooms'][501] = -60

	map.main()
