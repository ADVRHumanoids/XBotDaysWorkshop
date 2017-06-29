import rospy
import tf
import geometry_msgs.msg
import math

if __name__ == '__main__':

    rospy.init_node('tf_example')

    listener = tf.TransformListener()
    publisher_l = rospy.Publisher('w_T_left_ee', geometry_msgs.msg.PoseStamped, queue_size=1)
    publisher_r = rospy.Publisher('w_T_right_ee', geometry_msgs.msg.PoseStamped, queue_size=1)

    rate = rospy.Rate(100.0)

    tf_received = False


    trans_left = geometry_msgs.msg.Vector3
    rot_left = geometry_msgs.msg.Quaternion

    while not (rospy.is_shutdown() or tf_received):
        try:
            (trans_left, rot_left) = listener.lookupTransform('pelvis', 'arm1_7', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

        tf_received = True


    trans_left = geometry_msgs.msg.Vector3(trans_left[0], trans_left[1], trans_left[2])
    rot_left = geometry_msgs.msg.Quaternion(rot_left[0], rot_left[1], rot_left[2], rot_left[3])

    print(trans_left)
    print(rot_left)


    tf_received = False


    trans_right = geometry_msgs.msg.Vector3
    rot_right = geometry_msgs.msg.Quaternion

    while not (rospy.is_shutdown() or tf_received):
        try:
            (trans_right, rot_right) = listener.lookupTransform('pelvis', 'arm2_7', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

        tf_received = True


    trans_right = geometry_msgs.msg.Vector3(trans_right[0], trans_right[1], trans_right[2])
    rot_right = geometry_msgs.msg.Quaternion(rot_right[0], rot_right[1], rot_right[2], rot_right[3])

    print(trans_right)
    print(rot_right)

    while not rospy.is_shutdown():

        pos_left = geometry_msgs.msg.Vector3(trans_left.x, trans_left.y, trans_left.z)

        pose_ref_left = geometry_msgs.msg.PoseStamped()
        pose_ref_left.pose.position = pos_left
        pose_ref_left.pose.orientation = rot_left
        pose_ref_left.header.stamp = rospy.Time.now()
        pose_ref_left.pose.position.z = trans_left.z +  0.2 * math.sin(rospy.Time.now().to_sec()) + 0.2

        publisher_l.publish(pose_ref_left)

        pos_right = geometry_msgs.msg.Vector3(trans_right.x, trans_right.y, trans_right.z)

        pose_ref_right = geometry_msgs.msg.PoseStamped()
        pose_ref_right.pose.position = pos_right
        pose_ref_right.pose.orientation = rot_right
        pose_ref_right.header.stamp = rospy.Time.now()
        pose_ref_right.pose.position.z = trans_right.z +  0.2 * math.sin(rospy.Time.now().to_sec()) + 0.2

        publisher_r.publish(pose_ref_right)








