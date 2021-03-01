import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

class tf:
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber('/DCtransform', PoseStamped, self.publish, queue_size=1)
		while not rospy.is_shutdown():
			pass

	def publish(self, data):
		broadcaster = tf2_ros.StaticTransformBroadcaster()

		static_transformStamped = TransformStamped()
		static_transformStamped.header.stamp = rospy.Time.now()
		static_transformStamped.header.frame_id = 'station'
		static_transformStamped.child_frame_id = 'world'

		static_transformStamped.transform.translation.x = data.pose.position.x
		static_transformStamped.transform.translation.y = data.pose.position.y
		static_transformStamped.transform.translation.z = data.pose.position.z

		static_transformStamped.transform.rotation.x = data.pose.orientation.x
		static_transformStamped.transform.rotation.y = data.pose.orientation.y
		static_transformStamped.transform.rotation.z = data.pose.orientation.z
		static_transformStamped.transform.rotation.w = data.pose.orientation.w

		broadcaster.sendTransform(static_transformStamped)
		print 'spinning'
		rospy.spin() 


if __name__ == '__main__':
	tf()