import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from skspatial.objects import Points, Plane
from skspatial.transformation import transform_coordinates

class dimensionConverter:
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		self.publisher = rospy.Publisher('/DCtransform', PoseStamped, queue_size=1)
		self.robotListener = rospy.Subscriber('/robotTracker/pose', PoseStamped, self.trackerCallback, queue_size=1)
		self.agentListener = rospy.Subscriber('/DCcontrol', String, self.DCcontrolCallback, queue_size=1)

		self.point = None
		self.origin = None
		self.vector = None
		self.points = []
		self.vectorBasis = None

		while not rospy.is_shutdown():
			pass

	def trackerCallback(self, data):
		self.point = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

	def DCcontrolCallback(self, data):
		if data == 'point': # data.data?
			self.points.append(self.point.copy())
		elif data == 'origin':
			self.points.append(self.point.copy())
			self.origin = np.array(self.point)
		elif data == 'vector':
			self.points.append(self.point.copy())
			self.vector = np.array(self.point)
		elif data == 'compute':
			self.compute()
			self.publish()

	def compute(self):
		if len(self.points) < 3 or self.origin is None or self.vector is None:
			return 

		points = Points(self.points)
		self.plane = Plane.best_fit(points)
		self.vector = np.array(plane.project_point(self.vector))
		self.origin = np.array(plane.project_point(self.origin))
		self.xAxis = self.normalizeVector(self.vector - self.origin)
		self.zAxis = self.normalizeVector(np.array(plane.normal))
		self.yAxis = -np.cross(xAxis, zAxis)
		self.vectorBasis = [xAxis, yAxis, zAxis]
		self.quaternion = self.get_quaternion([[1, 0, 0], [0, 1, 0], [0, 0, 1]], self.vectorBasis)

	def publish(self)
		msg = PoseStamped()
		msg.header.frame_id = 'mapsim'
		msg.pose.position.x = self.origin[0]
		msg.pose.position.y = self.origin[1]
		msg.pose.position.z = self.origin[2]
		msg.pose.orientation.w = self.quaternion[0]
		msg.pose.orientation.x = self.quaternion[1]
		msg.pose.orientation.y = self.quaternion[2]
		msg.pose.orientation.z = self.quaternion[3]
		self.publisher.publish(msg)

	def normalizeVector(self, vector):
		norm = numpy.linalg.norm(vector)
		vector /= norm
		return vector

	def get_quaternion(self, lst1, lst2, matchlist=None):
		if not matchlist:
			matchlist=range(len(lst1))
		M=np.matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

		for i,coord1 in enumerate(lst1):
			x = np.matrix(np.outer(coord1, lst2[matchlist[i]]))
			M = M + x

		N11 = float(M[0][:, 0] + M[1][:, 1] + M[2][:, 2])
		N22 = float(M[0][:, 0] - M[1][:, 1] - M[2][:, 2])
		N33 = float(-M[0][:, 0] + M[1][:, 1] - M[2][:, 2])
		N44 = float(-M[0][:, 0] - M[1][:, 1] + M[2][:, 2])
		N12 = float(M[1][:, 2] - M[2][:, 1])
		N13 = float(M[2][:, 0] - M[0][:, 2])
		N14 = float(M[0][:, 1] - M[1][:, 0])
		N21 = float(N12)
		N23 = float(M[0][:, 1] + M[1][:, 0])
		N24 = float(M[2][:, 0] + M[0][:, 2])
		N31 = float(N13)
		N32 = float(N23)
		N34 = float(M[1][:, 2] + M[2][:, 1])
		N41 = float(N14)
		N42 = float(N24)
		N43 = float(N34)

		N = np.matrix([	[N11, N12, N13, N14], \
						[N21, N22, N23, N24], \
						[N31, N32, N33, N34], \
						[N41, N42, N43, N44]])


		values,vectors = np.linalg.eig(N)
		w = list(values)
		mw = max(w)
		quat = vectors[:, w.index(mw)]
		quat = np.array(quat).reshape(-1,)
		return quat

	# def trackerCallback2(self, data):
	# 	if self.origin is None or self.vectorBasis is None:
	# 		return
	# 	point = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
	# 	qvector = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
	# 	w = data.pose.orientation.w

	# 	point = transform_coordinates(point, self.origin, self.vectorBasis)
	# 	qvector = transform_coordinates(qvector, self.origin, self.vectorBasis)
	# 	x = (1 - w**2)/(qvector[0]**2 + qvector[1]**2 + qvector[2]**2)
	# 	qvector *= x

	# 	return point, qvector, w


if __name__ == '__main__':
	dimensionConverter()

	# points = [[0, 0, 0], [1, 0, 1], [0, 1, 1]]
	# origin = [0, 0, 0]

	# plane = Plane.best_fit(points)

	# vector = np.array(plane.project_point([1, 1, 1]))

	# xAxis = normalizeVector(vector - origin)
	# zAxis = normalizeVector(np.array(plane.normal))
	# yAxis = -np.cross(xAxis, zAxis)
	# vectorBasis = [xAxis, yAxis, zAxis]

	# w = 0.89
	# alpha = np.sin(np.arccos(w))
	# qvector = np.array([0.36, 0.27, 0])/alpha
	# qvector = np.array(transform_coordinates(qvector, origin, vectorBasis))
	# qvector *= alpha
	# x = (1 - w**2)/(qvector[0]**2 + qvector[1]**2 + qvector[2]**2)
	# qvector *= x

	# q = get_quaternion([[1, 0, 0], [0, 1, 0], [0, 0, 1]], vectorBasis)

	# print (q)
