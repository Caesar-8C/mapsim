import copy

def pickCorridors(self):
	corridors = []
	if len(self.nodePath) == 0 and not self.robot.edge is None:
		corridors.append(self.robot.edge)
		return corridors

	node = self.nodePath[0]
	for edge in self.G.edges:
		if node in edge:
			corridors.append(edge)
	return corridors

def removeAgents(self, corridors):
	for agentIndex in list(self.agents):
		deleteFlag = True
		for corridor in corridors:
			if self.agents[agentIndex].current_node in corridor and self.agents[agentIndex].next_node in corridor:
				deleteFlag = False
		if deleteFlag:
			del self.agents[agentIndex]

def predictFuture(self):
	corridors = self.pickCorridors()
	if len(corridors) == 0:
		return

	for agent in self.agents:
		self.agents[agent].map = None

	agentsBackup = copy.deepcopy(self.agents)

	for agent in self.agents:
		self.agents[agent].map = self
		self.agents[agent].fastforward = 30
	for agent in agentsBackup:
		agentsBackup[agent].map = self



	self.removeAgents(corridors)
	distance = self.robot.distance + self.robot.direction*self.WAYPOINT_DISTANCE
	direction = self.robot.direction
	edge = self.robot.edge
	data = []
	for i in range(self.fps*self.FUTURE_PREDICTION_TIME):
		self.moveAgents()
		waypoint_data = []
		distance += self.robot.direction*self.WAYPOINT_DISTANCE
		for agent in self.agents:
			if agent.current_node in edge and agent.next_node in edge and \
				direction*agent.distance < direction*distance:
				waypoint_data.append(agent.lane)
		data.append(waypoint_data)
	self.agents = agentsBackup

	return data