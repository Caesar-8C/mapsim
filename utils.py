import numpy as np

def tupleSum(t1, t2):
	return (t1[0] + t2[0], t1[1] + t2[1])

def tupleSubtract(t1, t2):
	return (t1[0] - t2[0], t1[1] - t2[1])

def tupleDistance(t1, t2):
	return np.linalg.norm(tupleSubtract(t1, t2))

def between(midNumber, boundary1, boundary2):
	if boundary1 > boundary2:
		if boundary1 > midNumber and midNumber > boundary2:
			return True
	else:
		if boundary2 > midNumber and midNumber > boundary1:
			return True
	return False

def subtractLists(a, b):
	res = []
	for i in a:
		if not i in b:
			res.append(i)
	return res

class Room:
	def __init__(self, start, end, distance):
		self.start = start
		self.end = end
		self.distance = distance