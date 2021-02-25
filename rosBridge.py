# import rospy
import numpy as np
# from geometry_msgs.msg import PoseStamped

class Bridge:
	def __init__(self, map):
		self.map = map
		self.enabled = False
		# rospy.init_node('listener', anonymous=True)
		# self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		# self.robotListener = rospy.Subscriber('/robotTracker/pose', PoseStamped, self.robotCallback, queue_size=1)
		# self.agentListener = rospy.Subscriber('/agentTracker/pose', PoseStamped, self.agentCallback, queue_size=1)

	def enable(self):
		self.enabled = True
	def disable(self):
		self.enabled = False

	def publish(self, waypoint, angle):
		# msg = PoseStamped()
		# msg.header.frame_id = 'map'
		# msg.pose.position.x = waypoint[0]
		# msg.pose.position.y = waypoint[1]
		# msg.pose.orientation.z = np.sin(angle/2.)
		# msg.pose.orientation.w = np.cos(angle/2.)
		# self.publisher.publish(msg)
		pass

	def robotCallback(self, data):
		self.map.robot.x = data.pose.position.x
		self.map.robot.y = data.pose.position.y

		q = data.pose.orientation
		q.y = q.z
		q.z = 0.0

		self.map.robot.theta = self.quat2Rotation(q)

	def quat2Rotation(self, q):
		angle = -np.arcsin(2*(q.x*q.z - q.w*q.y))
		robotFacingBackwards = 1-2*(q.y**2 + q.z**2) < 0
		if robotFacingBackwards:
			if angle < 0:
				angle = -(np.pi + angle)
			else:
				angle = np.pi - angle
		return angle

	def agentCallback(self, data):
		# TODO somehow set an identifier for the agent
		identifier = 0
		agent = self.map.agents[identifier]
		agent.x = data.pose.position.x
		agent.y = data.pose.position.y
		agent.coordinates = (agent.x, agent.y)

		if 	   (agent.direction > 0 and agent.distance >= agent.maxdist)\
			or (agent.direction < 0 and agent.distance <= 0):
			agent.corridorEndReached()
		else:
			agent.computeCorridorDistances()