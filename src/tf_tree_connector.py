#!/usr/bin/env python

# Import required Python code.
import rospy
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import transform_to_kdl


# Import custom message data.
from std_msgs.msg import Empty

def do_transform_transform(trafo, transform):
    f = transform_to_kdl(transform) *transform_to_kdl(trafo)
    res = geometry_msgs.msg.TransformStamped()
    res.transform.translation.x = f[(0, 3)]
    res.transform.translation.y = f[(1, 3)]
    res.transform.translation.z = f[(2, 3)]
    (res.transform.rotation.x, res.transform.rotation.y, res.transform.rotation.z, res.transform.rotation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res


class TF2TreeConnector(object):
    def __init__(self):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reference_map_frame = rospy.get_param('~reference_map_frame', 'map_stargazer')
        self.reference_base_link_frame = rospy.get_param('~reference_base_link', 'cam_stargazer')
        self.map_frame = rospy.get_param('~map_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link', 'vehicle')
        self.lookup_rate = rospy.get_param('~lookup_rate', 100.0)

    def connect_trees(self, msg=None):
        rate = rospy.Rate(self.lookup_rate)
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform(self.reference_map_frame, self.reference_base_link_frame, rospy.Time())
                trans = do_transform_transform(trans, self.tf_buffer.lookup_transform(self.base_link_frame, self.map_frame, rospy.Time()))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo_throttle(1, "No trafo found!")
                rate.sleep()
                continue
            rate.sleep()

        rospy.loginfo("Trafo found!")
        trans.header.frame_id = self.reference_map_frame
        trans.child_frame_id = self.map_frame
        self.broadcaster.sendTransform(trans)

if __name__ == '__main__':
    rospy.init_node('tf_tree_connector', anonymous=True)
    TREE_CONNECTOR = TF2TreeConnector()
    TREE_CONNECTOR.connect_trees()
    RESET_SUBSCRIBER = rospy.Subscriber("~reset", Empty, TREE_CONNECTOR.connect_trees)
    rospy.spin()
