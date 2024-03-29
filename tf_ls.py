import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped, Vector3
from nav_msgs.msg import Odometry
import tf

class tf_ls:
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		self.robotPublisher = rospy.Publisher('/tracker/world_odometry1', Odometry, queue_size=1)
		self.agentPublisher = rospy.Publisher('/tracker/world_odometry2', Odometry, queue_size=1)
		listener =  rospy.Subscriber('/tracker/odometry1', Odometry, self.robotCallback, queue_size=1)
		listener =  rospy.Subscriber('/tracker/odometry2', Odometry, self.agentCallback, queue_size=1)
		self.tl = tf.TransformListener()
		rospy.spin()

	def transform(self, data):
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose

		vector = Vector3Stamped()
		vector.header = data.header
		vector.vector = data.twist.twist.linear

		try:
			pose = self.tl.transformPose('world', pose)
			vector = self.tl.transformVector3('world', vector)
		except:
			print 'no transform to world frame'
			return

		msg = Odometry()
		msg.header = data.header
		msg.header.frame_id = 'world'
		msg.child_frame_id = data.child_frame_id
		
		msg.pose.pose = pose.pose
		msg.twist.twist.linear = vector.vector

		return msg


	def robotCallback(self, data):
		# msg = self.transform(data)
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose

		vector = Vector3Stamped()
		vector.header = data.header
		vector.vector = data.twist.twist.linear

		try:
			pose = self.tl.transformPose('world', pose)
			vector = self.tl.transformVector3('world', vector)
		except:
			print 'no transform to world frame'
			return

		msg = Odometry()
		msg.header = data.header
		msg.header.frame_id = 'world'
		msg.child_frame_id = data.child_frame_id
		
		msg.pose.pose = pose.pose
		msg.twist.twist.linear = vector.vector
		self.robotPublisher.publish(msg)

	def agentCallback(self, data):
		# msg = self.transform(data)
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose

		vector = Vector3Stamped()
		vector.header = data.header
		vector.vector = data.twist.twist.linear

		try:
			pose = self.tl.transformPose('world', pose)
			vector = self.tl.transformVector3('world', vector)
		except:
			print 'no transform to world frame'
			return

		msg = Odometry()
		msg.header = data.header
		msg.header.frame_id = 'world'
		msg.child_frame_id = data.child_frame_id
		
		msg.pose.pose = pose.pose
		msg.twist.twist.linear = vector.vector
		self.agentPublisher.publish(msg)


if __name__ == '__main__':
	tf_ls()