import networkx as nx

def loadGraph(self, file):
	self.clearData()

	f = open(file)
	if f.closed:
		print('file failed to open')
		return

	for line in f:
		words = line.split()
		if len(words) == 0:
			continue
		if words[0] == 'scale' and len(words) > 1:
			self.draw.scale = float(words[1])
		if words[0] == 'shift' and len(words) > 2:
			self.draw.shift = (int(words[1]), int(words[2]))
		if words[0] == 'node' and len(words) > 3:
			self.add_node(name=int(words[1]), x=int(words[2]), y=int(words[3]))
		if words[0] == 'edge' and len(words) > 3:
			self.add_edge(start=int(words[1]), end=int(words[2]), width=int(words[3]))

		if words[0] == 'room' and len(words) > 4:
			self.add_room(number=int(words[1]), start=int(words[2]), end=int(words[3]), distance=int(words[4]))

	f.close()

	self.calculatePathMap()

def saveGraph(self, file):
	f = open(file, 'w')
	if f.closed:
		print('file failed to open')
		return

	f.write('scale '+str(self.draw.scale)+'\n')
	f.write('shift '+str(self.draw.shift[0])+' '+str(self.draw.shift[1])+'\n\n')

	for node in self.G.nodes:
		x, y = self.G.nodes[node]['coordinates']
		f.write('node '+str(node)+' '+str(x)+' '+str(y)+'\n')
	f.write('\n')

	for edge in self.G.edges:
		f.write('edge '+str(edge[0])+' '+str(edge[1])+' '+str(self.G.edges[edge]['width'])+'\n')
	f.write('\n')
	
	for room in self.rooms:
		f.write('room '+str(room)+' '+str(self.rooms[room].start)+' '+str(self.rooms[room].end)+' '+str(self.rooms[room].distance)+'\n')

	f.close()

def clearData(self):
	self.G = nx.Graph()
	self.rooms = {}
	self.agents = {}
	self.robot.reset()