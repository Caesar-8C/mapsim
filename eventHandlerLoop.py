import pygame as pyg
from utils import *
import sys

def mouseScale(self, rel):
	rel = tupleSum((rel[0]/self.draw.scale, rel[1]/self.draw.scale), self.rel_residual)
	self.rel_residual = (rel[0]-int(rel[0]), rel[1]-int(rel[1]))
	return (int(rel[0]), int(rel[1]))
	
def mouseDescale(self, pos):
	return (int(pos[0]/self.draw.scale-self.draw.shift[0]), int(pos[1]/self.draw.scale-self.draw.shift[1]))

def eventHandlerLoop(self):
	for event in pyg.event.get():
		if event.type == pyg.QUIT:
			sys.exit()
		if event.type == pyg.MOUSEBUTTONDOWN and event.button == 1:
			self.activateElement(self.mouseDescale(event.pos))
		if event.type == pyg.MOUSEBUTTONUP and event.button == 1:
			self.rel_residual = (0, 0)
			self.activeReset()

		if event.type == pyg.MOUSEMOTION:
			rel = self.mouseScale(event.rel)
		if event.type == pyg.MOUSEMOTION and not self.activeNode == False:
			self.G.nodes[self.activeNode]['coordinates'] = tupleSum(self.G.nodes[self.activeNode]['coordinates'], rel)
			self.mapChanged()
		if event.type == pyg.MOUSEMOTION and not self.activeEdge == False:
			self.G.nodes[self.activeEdge[0]]['coordinates'] = tupleSum(self.G.nodes[self.activeEdge[0]]['coordinates'], rel)
			self.G.nodes[self.activeEdge[1]]['coordinates'] = tupleSum(self.G.nodes[self.activeEdge[1]]['coordinates'], rel)
			self.mapChanged()

		if event.type == pyg.MOUSEMOTION and self.activeBackground == True:
			self.draw.shift = tupleSum(self.draw.shift, rel)


		if event.type == pyg.MOUSEBUTTONDOWN and event.button == 4:
			self.activateElement(self.mouseDescale(event.pos))
			if not self.activeEdge == False:
				self.G.edges[self.activeEdge]['width'] += 4
			if self.activeBackground == True:
				pos_1 = self.mouseDescale(event.pos)
				self.draw.scale *= 1.1
				self.draw.shift = tupleSum(self.draw.shift, tupleSubtract(self.mouseDescale(event.pos), pos_1))
			self.activeReset()
			self.mapChanged()
		if event.type == pyg.MOUSEBUTTONDOWN and event.button == 5:
			self.activateElement(self.mouseDescale(event.pos))
			if not self.activeEdge == False:
				self.G.edges[self.activeEdge]['width'] -= 4
				if self.G.edges[self.activeEdge]['width'] < 1:
					self.G.edges[self.activeEdge]['width'] = 1
			if self.activeBackground == True:
				pos_1 = self.mouseDescale(event.pos)
				self.draw.scale /= 1.1
				self.draw.shift = tupleSum(self.draw.shift, tupleSubtract(self.mouseDescale(event.pos), pos_1))
			self.activeReset()
			self.mapChanged()

		if event.type == pyg.KEYDOWN:
			if event.key == pyg.K_k:
				self.saveGraph('data/graph7.txt')
			if event.key == pyg.K_l:
				self.loadGraph('data/graph2.txt')


			if event.key == pyg.K_c:
				self.run_agent()

		if event.type == self.AGENT1:
			self.run_agent(origin_node=2, end_node=3, lane=1, speed=60)
		if event.type == self.AGENT2:
			self.run_agent(origin_node=3, end_node=11, lane=0, speed=50)
		if event.type == self.AGENT3:
			self.run_agent(origin_node=2, end_node=3, lane=0, speed=-60)


		# ROBOT

		if event.type == pyg.KEYDOWN:
			if event.key == pyg.K_w:
				self.controlAction[0] = 1.2
			if event.key == pyg.K_s:
				self.controlAction[0] = -1.2
			if event.key == pyg.K_e:
				self.controlAction[1] = 1.2
			if event.key == pyg.K_q:
				self.controlAction[1] = -1.2
			if event.key == pyg.K_d:
				self.controlAction[2] = 0.02
			if event.key == pyg.K_a:
				self.controlAction[2] = -0.02

		if event.type == pyg.KEYUP:
			if event.key == pyg.K_w or event.key == pyg.K_s:
				self.controlAction[0] = 0
			if event.key == pyg.K_q or event.key == pyg.K_e:
				self.controlAction[1] = 0
			if event.key == pyg.K_a or event.key == pyg.K_d:
				self.controlAction[2] = 0