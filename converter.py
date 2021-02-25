import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from skspatial.objects import Points, Plane
from skspatial.transformation import transform_coordinates

class Position():
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
class Orientation():
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
		self.w = 0
class Pose():
	def __init__(self):
		self.position = Position()
		self.orientation = Orientation()

class dimensionConverter:
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.robotListener = rospy.Subscriber('/robotTracker/pose', PoseStamped, self.trackerCallback, queue_size=1)
		self.agentListener = rospy.Subscriber('/DCcontrol', String, self.DCcontrolCallback, queue_size=1)

		self.pose = Pose()
		self.origin = None
		self.points = []

		while not rospy.is_shutdown():
			pass

	def trackerCallback(self, data):
		self.pose.position.x = data.position.x
		self.pose.position.y = data.position.y
		self.pose.position.z = data.position.z
		self.pose.orientation.x = data.orientation.x
		self.pose.orientation.y = data.orientation.y
		self.pose.orientation.z = data.orientation.z
		self.pose.orientation.w = data.orientation.w

	def DCcontrolCallback(self, data):
		if data == 'point':
			self.points.append([self.pose.position.x, self.pose.position.y, self.pose.position.z])
		elif data == 'origin':
			self.origin = self.pose.copy()
		elif data == 'compute':
			self.compute()

	def compute():
		if len(self.points) < 3 or self.origin is None:
			return 

		points = Points(self.points)
		self.plane = Plane.best_fit(points)
		# compute a vector in origin's direction
		normal = plane.normal
		# compute a cross product vector
		# find transformation from steam to plane

		self.vectorBasis = None
		self.origin = [self.origin.position.x, self.origin.position.y, self.origin.position.z]
		return self.origin, self.vectorBasis
		transform_coordinates(point, self.origin, vectorBasis)

if __name__ == '__main__':
	dimensionConverter()