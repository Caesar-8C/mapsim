import copy

def pickCorridors(self):
	node = self.nodePath[0]
	corridors = []
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
	# agentsBackup = self.agents.copy()

	for agent in self.agents:
		self.agents[agent].map = None

	agentsBackup = copy.deepcopy(self.agents)

	for agent in self.agents:
		self.agents[agent].map = self
		self.agents[agent].fastforward = 30
	for agent in agentsBackup:
		agentsBackup[agent].map = self



	self.removeAgents(corridors)
	for i in range(self.fps*self.FUTURE_PREDICTION_TIME):
		self.moveAgents()
	# TODO gather the data, where agents end up
	self.agents = agentsBackup