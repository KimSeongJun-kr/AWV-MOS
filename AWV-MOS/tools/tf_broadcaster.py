import rospy
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg

def handle_odometry(msg):
    # TF 브로드캐스터 생성
    br = tf2_ros.TransformBroadcaster()

    tf_msg = geometry_msgs.msg.TransformStamped()
    tf_msg.header.stamp = msg.header.stamp
    tf_msg.header.frame_id = msg.header.frame_id
    tf_msg.child_frame_id = msg.child_frame_id
    tf_msg.transform.translation.x = msg.pose.pose.position.x
    tf_msg.transform.translation.y = msg.pose.pose.position.y
    tf_msg.transform.translation.z = msg.pose.pose.position.z
    tf_msg.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(tf_msg)

    static_tf_msg = geometry_msgs.msg.TransformStamped()
    static_tf_msg.header.stamp = msg.header.stamp
    static_tf_msg.header.frame_id = msg.child_frame_id
    static_tf_msg.child_frame_id = "velodyne"
    static_tf_msg.transform.rotation.w = 1.0
    br.sendTransform(static_tf_msg)

if __name__ == '__main__':
    rospy.init_node('odometry_to_tf')

    rospy.Subscriber('/suma_pose/odom', Odometry, handle_odometry)
    # rospy.Subscriber('/gt_pose/odom', Odometry, handle_odometry)

    rospy.spin()
