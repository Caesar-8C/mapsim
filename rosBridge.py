import rospy
import numpy as np
from nav_msgs.msg import Odometry

class Bridge:
	def __init__(self, map):
		self.map = map
		self.enabled = False
		rospy.init_node('listener', anonymous=True)
		self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.robotListener = rospy.Subscriber('/tracker/world_odometry', Odometry, self.robotCallback, queue_size=1)
		# self.agentListener = rospy.Subscriber('/tracker/world_odometry', Odometry, self.agentCallback, queue_size=1)
		self.scale = 40

	def enable(self):
		self.enabled = True
	def disable(self):
		self.enabled = False

	def publish(self, waypoint, angle):
		msg = PoseStamped()
		msg.header.frame_id = 'world'
		msg.pose.position.x = waypoint[0]/self.scale
		msg.pose.position.y = waypoint[1]/self.scale
		msg.pose.orientation.z = np.sin(angle/2.)
		msg.pose.orientation.w = np.cos(angle/2.)
		self.publisher.publish(msg)

	def normalizeAngle(self, angle):
		while angle < -np.pi:
			angle += 2*np.pi
		while angle > np.pi:
			angle -= 2*np.pi
		return angle

	def robotCallback(self, data):
		self.map.robot.x = data.pose.pose.position.x * self.scale
		self.map.robot.y = data.pose.pose.position.y * self.scale


		x_vel = data.twist.twist.linear.x
		y_vel = data.twist.twist.linear.y
		self.map.robot.addToBridgeRunningVelocity(np.abs(x_vel) + np.abs(y_vel))


		q = data.pose.pose.orientation
		self.map.robot.theta = self.normalizeAngle(self.quat2Rotation(q) - np.pi/2)

	def quat2Rotation(self, q):
		a = 2*(q.w*q.z + q.x*q.y)
		b = 1 - 2*(q.y**2 + q.z**2)
		angle = np.arctan2(a, b)
		return angle

	def agentCallback(self, data):
		# TODO somehow set an identifier for the agent
		identifier = 0
		agent = self.map.agents[identifier]
		agent.x = data.pose.pose.position.x
		agent.y = data.pose.pose.position.y
		agent.coordinates = (agent.x, agent.y)

		x_vel = data.twist.twist.linear.x
		y_vel = data.twist.twist.linear.y
		agent.addToBridgeRunningVelocity(np.abs(x_vel) + np.abs(y_vel))

		if 	   (agent.direction > 0 and agent.distance >= agent.maxdist)\
			or (agent.direction < 0 and agent.distance <= 0):
			agent.corridorEndReached()
		else:
			agent.computeCorridorDistances()