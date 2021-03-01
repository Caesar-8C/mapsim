import copy
import numpy as np

def pickCorridors(self):
	corridors = []
	if len(self.nodePath) == 0:
		if not self.robot.edge is None:
			corridors.append(self.robot.edge)
		return corridors

	node = self.nodePath[0]
	for edge in self.G.edges:
		if node in edge:
			corridors.append(edge)
	return corridors

def getActiveAgents(self, corridors):
	activeAgentsIndices = []
	for agent in self.agents:
		for corridor in corridors:
			if self.agents[agent].current_node in corridor and self.agents[agent].next_node in corridor:
				activeAgentsIndices.append(agent)
	return activeAgentsIndices

def backupAgents(self, corridors):
	copyIndices = self.getActiveAgents(corridors)
	copiedAgents = {}
	for agent in self.agents:
		self.agents[agent].map = None
		if agent in copyIndices:
			copiedAgent = copy.deepcopy(self.agents[agent])
			copiedAgent.map = self
			copiedAgent.fastforward = self.FAST_FORWARD
			copiedAgents[agent] = copiedAgent
		self.agents[agent].map = self

	agentsBackup = self.agents
	self.agents = copiedAgents
	return agentsBackup

def getCurrentEdge(self, targetEdge):
	if not self.robot.edge is None:
		edge = self.robot.edge
	elif not self.robot.node is None:
		if len(self.nodePath) < 2:
			edge = targetEdge
		else:
			edge = (self.robot.node, self.nodePath[1])
	else:
		edge = None
	return edge

def getPreviousDistance(self, edge, direction, stepDist):
	maxDist = self.G.edges[edge]['length']
	distance =  0 if direction == 1 else maxDist

	if self.robot.distance is None:
		targetDist = 0 if direction == 1 else maxDist
	else:
		targetDist = self.robot.distance

	while direction*distance < direction*(targetDist - direction*self.PREDICTION_MARGIN):
		distance += direction*stepDist
	previousDistance = distance
	return previousDistance

def getNextEdge(self, targetEdge, nodePathCounter):
	if len(self.nodePath) < nodePathCounter+2:
		edge = targetEdge
	else:
		edge = (self.nodePath[nodePathCounter], self.nodePath[nodePathCounter+1])
		nodePathCounter += 1
	return edge

def getDirection(self, edge, targetEdge, nodePathCounter):
	if edge != targetEdge:
		direction = 1 if self.nodePath[nodePathCounter] == self.G.edges[edge]['end'] else -1
	else:
		if self.robot.distance is None:
			if self.robot.node == self.G.edges[targetEdge]['start']:
				direction = 1
			else:
				direction = -1
		elif self.robot.distance < abs(self.rooms[self.target].distance):
			direction = 1
		else:
			direction = -1
	return direction

def predictFuture(self):
	corridors = self.pickCorridors()
	if len(corridors) == 0:
		return

	agentsBackup = self.backupAgents(corridors)

	targetEdge = (self.rooms[self.target].start, self.rooms[self.target].end)
	edge = self.getCurrentEdge(targetEdge)
	if edge is None:
		self.agents = agentsBackup
		return []


	nodePathCounter = 0
	direction = self.getDirection(edge, targetEdge, nodePathCounter)

	velocity = np.mean(self.robot.bridgeRunningVelocity) if self.bridge.enabled else self.robot.maxVelocity[0]
	stepDist = velocity*self.FAST_FORWARD
	previousDistance = self.getPreviousDistance(edge, direction, stepDist)
	distance = previousDistance + direction*(stepDist + self.PREDICTION_MARGIN)

	data = []
	for i in range(int(self.fps*self.FUTURE_PREDICTION_TIME/self.FAST_FORWARD)):
		self.moveAgents()
		waypoint_data = []
		for agent in self.agents:
			if self.agents[agent].current_node in edge and self.agents[agent].next_node in edge and \
				direction*self.agents[agent].distance < direction*distance and \
				direction*self.agents[agent].distance > direction*previousDistance:
				if not self.agents[agent].lane in waypoint_data: waypoint_data.append(self.agents[agent].lane)
		data.append(waypoint_data)

		if distance < 0 or distance > self.G.edges[edge]['length']:
			if edge == targetEdge:
				break
			nodePathCounter += 1
			edge = self.getNextEdge(targetEdge, nodePathCounter)

			direction = self.getDirection(edge, targetEdge, nodePathCounter)
			maxDist = self.G.edges[edge]['length']
			previousDistance = 0 if direction == 1 else maxDist
			distance = previousDistance + direction*(stepDist + self.PREDICTION_MARGIN)
			continue

		previousDistance += direction*stepDist
		distance += direction*stepDist

	self.agents = agentsBackup
	return data