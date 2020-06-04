def tupleSum(t1, t2):
	return (t1[0] + t2[0], t1[1] + t2[1])

def tupleSubtract(t1, t2):
	return (t1[0] - t2[0], t1[1] - t2[1])

def between(a, b, c):
	if b > c:
		if b > a and a > c:
			return True
	else:
		if c > a and a > b:
			return True
	return False

class Agent:
	def __init__(self, origin_node, end_node, lane, speed):
		self.origin_node = origin_node
		self.end_node = end_node
		self.lane = lane
		self.speed = speed
		self.distance = 0

class Room:
	def __init__(self, start, end, distance):
		self.start = start
		self.end = end
		self.distance = distance