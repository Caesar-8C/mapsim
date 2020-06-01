import numpy as np
import pygame as pyg
import networkx as nx
import sys
import time

class Draw:
	def __init__(self, scale=1.5, shift=(0, 0)):
		self.scale = scale
		self.shift = shift

	def line(self, surface, color, start_pos, end_pos, width):
		start_pos = (int((start_pos[0]+self.shift[0]) * self.scale), int((start_pos[1]+self.shift[1]) * self.scale))
		end_pos = (int((end_pos[0]+self.shift[0]) * self.scale), int((end_pos[1]+self.shift[1]) * self.scale))
		pyg.draw.line(surface, color, start_pos, end_pos, width)

	def circle(self, surface, color, center, radius):
		center = (int((center[0]+self.shift[0]) * self.scale), int((center[1]+self.shift[1]) * self.scale))
		radius = int(radius*self.scale)
		pyg.draw.circle(surface, color, center, radius)

	def polygon(self, surface, color, points):
		p = []
		for point in points:
			p.append((int((point[0]+self.shift[0]) * self.scale), int((point[1]+self.shift[1]) * self.scale)))
		pyg.draw.polygon(surface, color, tuple(p))


class Map:
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

		self.activeNode = False
		self.activeEdge = False
		self.activeBackground = False

		self.NODE_SIZE = 10
		self.MIN_CLICKABLE_CORRIDOR_WIDTH = 5
		self.LANE_PIXEL_WIDTH = 20

		self.AGENT1 = pyg.USEREVENT+1
		self.AGENT2 = pyg.USEREVENT+2
		self.AGENT3 = pyg.USEREVENT+3
		pyg.time.set_timer(self.AGENT1, 1000)
		pyg.time.set_timer(self.AGENT2, 1200)
		pyg.time.set_timer(self.AGENT3, 4000)

	class Agent:
		def __init__(self, origin_node, end_node, lane, speed):
			self.origin_node = origin_node
			self.end_node = end_node
			self.lane = lane
			self.speed = speed
			self.distance = 0

	def run_agent(self, origin_node, end_node, lane, speed):
		agent = self.Agent(origin_node, end_node, lane, speed)
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

	def main(self):
		def tupleSum(t1, t2):
			return (t1[0] + t2[0], t1[1] + t2[1])
		def tupleSubtract(t1, t2):
			return (t1[0] - t2[0], t1[1] - t2[1])
		def descaleMouse(pos):
			return (int(pos[0]/self.draw.scale-self.draw.shift[0]), int(pos[1]/self.draw.scale-self.draw.shift[1]))

		def activateElement(pos):
			def between(a, b, c):
				if b > c:
					if b > a and a > c:
						return True
				else:
					if c > a and a > b:
						return True
				return False

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

		def mouseScale(rel):
			return (int(rel[0]/self.draw.scale), int(rel[1]/self.draw.scale))

		def eventHandlerLoop():
			for event in pyg.event.get():
				if event.type == pyg.QUIT:
					sys.exit()
				if event.type == pyg.MOUSEBUTTONDOWN and event.button == 1:
					activateElement(descaleMouse(event.pos))
				if event.type == pyg.MOUSEBUTTONUP and event.button == 1:
					self.activeReset()

				if event.type == pyg.MOUSEMOTION and not self.activeNode == False:
					self.G.nodes[self.activeNode]['coordinates'] = tupleSum(self.G.nodes[self.activeNode]['coordinates'], mouseScale(event.rel))
				if event.type == pyg.MOUSEMOTION and not self.activeEdge == False:
					self.G.nodes[self.activeEdge[0]]['coordinates'] = tupleSum(self.G.nodes[self.activeEdge[0]]['coordinates'], mouseScale(event.rel))
					self.G.nodes[self.activeEdge[1]]['coordinates'] = tupleSum(self.G.nodes[self.activeEdge[1]]['coordinates'], mouseScale(event.rel))


				if event.type == pyg.MOUSEMOTION and self.activeBackground == True:
					self.draw.shift = tupleSum(self.draw.shift, mouseScale(event.rel))

				if event.type == pyg.MOUSEBUTTONDOWN and event.button == 4:
					activateElement(descaleMouse(event.pos))
					if not self.activeEdge == False:
						self.G.edges[self.activeEdge]['width'] += 4
					if self.activeBackground == True:
						pos_1 = descaleMouse(event.pos)
						self.draw.scale *= 1.1
						self.draw.shift = tupleSum(self.draw.shift, tupleSubtract(descaleMouse(event.pos), pos_1))
					self.activeReset()
				if event.type == pyg.MOUSEBUTTONDOWN and event.button == 5:
					activateElement(descaleMouse(event.pos))
					if not self.activeEdge == False:
						self.G.edges[self.activeEdge]['width'] -= 4
						if self.G.edges[self.activeEdge]['width'] < 1:
							self.G.edges[self.activeEdge]['width'] = 1
					if self.activeBackground == True:
						pos_1 = descaleMouse(event.pos)
						self.draw.scale /= 1.1
						self.draw.shift = tupleSum(self.draw.shift, tupleSubtract(descaleMouse(event.pos), pos_1))
					self.activeReset()

				if event.type == self.AGENT1:
					self.run_agent(origin_node=2, end_node=3, lane=1, speed=60)
				if event.type == self.AGENT2:
					self.run_agent(origin_node=3, end_node=4, lane=0, speed=50)
				if event.type == self.AGENT3:
					self.run_agent(origin_node=2, end_node=3, lane=0, speed=-60)

		def drawRect(color, node, successor):
			x1 = self.G.nodes[node]['coordinates'][0]
			y1 = self.G.nodes[node]['coordinates'][1]
			x2 = self.G.nodes[successor]['coordinates'][0]
			y2 = self.G.nodes[successor]['coordinates'][1]
			halfWidth = self.G.edges[node, successor]['width']/2

			angle = np.arctan2(x1-x2, y2-y1)
			x3 = x1 + np.cos(angle)*halfWidth
			y3 = y1 + np.sin(angle)*halfWidth

			x4 = x1 - np.cos(angle)*halfWidth
			y4 = y1 - np.sin(angle)*halfWidth

			x5 = x2 - np.cos(angle)*halfWidth
			y5 = y2 - np.sin(angle)*halfWidth

			x6 = x2 + np.cos(angle)*halfWidth
			y6 = y2 + np.sin(angle)*halfWidth

			self.draw.polygon(self.screen, color, ((x3, y3), (x4, y4), (x5, y5), (x6, y6)))
		def drawRooms(color, node, successor):
			x1 = self.G.nodes[node]['coordinates'][0]
			y1 = self.G.nodes[node]['coordinates'][1]
			x2 = self.G.nodes[successor]['coordinates'][0]
			y2 = self.G.nodes[successor]['coordinates'][1]
			width = self.G.edges[node, successor]['width']

			angleAcross = np.arctan2(x1-x2, y2-y1)
			angleAlong = np.arctan2(y1-y2, x1-x2)
			for room in self.G.edges[node, successor]['rooms']:
				dist = self.G.edges[node, successor]['rooms'][room]
				x3 = x1 + np.cos(angleAlong)*dist - np.sign(dist)*np.cos(angleAcross)*width/2
				y3 = y1 + np.sin(angleAlong)*dist - np.sign(dist)*np.sin(angleAcross)*width/2

				x4 = x3 + np.cos(angleAlong)*20
				y4 = y3 + np.sin(angleAlong)*20

				self.draw.line(self.screen, color, (x3, y3), (x4, y4), 3)
		def drawLanes(color, node, successor):
			x1 = self.G.nodes[node]['coordinates'][0]
			y1 = self.G.nodes[node]['coordinates'][1]
			x2 = self.G.nodes[successor]['coordinates'][0]
			y2 = self.G.nodes[successor]['coordinates'][1]
			width = self.G.edges[node, successor]['width']
			
			lanes = int(width/self.LANE_PIXEL_WIDTH)
			if lanes == 0:
				lanes = 1
			self.G.edges[node, successor]['lanes'] = lanes
			laneWidth = width/lanes
			self.G.edges[node, successor]['laneWidth'] = laneWidth

			if lanes > 1:
				angleNorm = np.arctan2(x1-x2, y2-y1)
				angleCos = np.cos(angleNorm)
				angleSin = np.sin(angleNorm)

				x3 = x1 + angleCos*width/2
				y3 = y1 + angleSin*width/2

				x4 = x2 + angleCos*width/2
				y4 = y2 + angleSin*width/2

				xStep = angleCos*laneWidth
				yStep = angleSin*laneWidth

				for i in range(lanes-1):
					x3 -= xStep
					y3 -= yStep

					x4 -= xStep
					y4 -= yStep

					self.draw.line(self.screen, color, (x3, y3), (x4, y4), 1)
		def drawAgents(color):
			indicesToDelete = []
			for agentIndex in self.agents:
				#TODO add lane existence check
				#TODO add negative speed check
				agent = self.agents[agentIndex]
				x1 = self.G.nodes[agent.origin_node]['coordinates'][0]
				y1 = self.G.nodes[agent.origin_node]['coordinates'][1]
				x2 = self.G.nodes[agent.end_node]['coordinates'][0]
				y2 = self.G.nodes[agent.end_node]['coordinates'][1]
				laneWidth = self.G.edges[agent.origin_node, agent.end_node]['laneWidth']
				width = self.G.edges[agent.origin_node, agent.end_node]['width']

				angleAcross = np.arctan2(x1-x2, y2-y1)
				angleAlong = np.arctan2(y1-y2, x1-x2)

				x3 = int(np.cos(angleAcross)*(width/2 - agent.lane*laneWidth - laneWidth/2) - np.cos(angleAlong)*agent.distance)
				y3 = int(np.sin(angleAcross)*(width/2 - agent.lane*laneWidth - laneWidth/2) - np.sin(angleAlong)*agent.distance)

				if agent.speed > 0:
					x3 += x1
					y3 += y1
				else:
					x3 += x2
					y3 += y2

				self.draw.circle(self.screen, color, (x3, y3), int(laneWidth/2))

				agent.distance += agent.speed/self.fps

				maxdist = np.linalg.norm((y2-y1, x2-x1))

				if np.abs(agent.distance) >= maxdist:
					indicesToDelete.append(agentIndex)

			for i in indicesToDelete:
				del self.agents[i]


		def drawSuccessors(node, visited):
			visited.append(node)
			for successor in list(self.G.successors(node)):
				drawRect((255, 255, 255), node, successor)
				drawRooms((255, 0, 0), node, successor)
				drawLanes((0, 100, 0), node, successor)
				if successor not in visited:
					drawSuccessors(successor, visited)

			self.draw.circle(self.screen, (0, 128, 0), self.G.nodes[node]['coordinates'], self.NODE_SIZE)

		while True:
			self.screen.fill(self.bgcolour)
			self.clock.tick(self.fps)
			eventHandlerLoop()

		
			node = 1
			drawSuccessors(node, [])
			drawAgents((0, 0, 100))

			pyg.display.flip()


if __name__ == '__main__':
	map = Map()


	map.add_node(name=1, x=500, y=200)
	map.add_node(name=2, x=400, y=300)
	map.add_node(name=3, x=500, y=400)
	map.add_node(name=4, x=600, y=300)
	map.add_node(name=5, x=300, y=400)

	map.add_edge(start=1, end=2, width=25)
	map.add_edge(start=2, end=3, width=50)
	map.add_edge(start=3, end=4, width=25)
	map.add_edge(start=2, end=5, width=13)
	map.add_edge(start=4, end=1, width=13)

	map.G.edges[2, 3]['rooms'][501] = -60


	map.main()