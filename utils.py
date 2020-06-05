import numpy as np

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

class Robot:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.theta = 0

		self.longitudinalVelocity = 0
		self.lateralVelocity = 0
		self.angularVelocity = 0

	def changeVelocity(self, velocity, controlAction):
		if controlAction:
			velocity += controlAction
		else:
			if abs(velocity) > 1:
				velocity -= np.sign(velocity)*1 # radian
			else:
				velocity = 0
		return velocity

	def move(self, controlAction):
		self.x += self.longitudinalVelocity*np.cos(self.theta) + self.lateralVelocity*np.sin(self.theta)
		self.y += self.longitudinalVelocity*np.sin(self.theta) + self.lateralVelocity*np.cos(self.theta)
		self.theta += self.angularVelocity

		self.longitudinalVelocity = self.changeVelocity(self.longitudinalVelocity, controlAction[0])
		self.lateralVelocity = self.changeVelocity(self.lateralVelocity, controlAction[1])
		self.angularVelocity = self.changeVelocity(self.angularVelocity, controlAction[2])