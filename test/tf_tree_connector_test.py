#!/usr/bin/env python

import sys
import unittest
import tf2_ros
import rospy
import geometry_msgs.msg

class TfTreeConnectorTest(unittest.TestCase):
    def test(self):
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                base_link_to_base_link = tf_buffer.lookup_transform('base_link', 'reference_base_link', rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

        self.assertEquals(0, base_link_to_base_link.transform.translation.x)
        self.assertEquals(0, base_link_to_base_link.transform.translation.y)
        self.assertEquals(0, base_link_to_base_link.transform.translation.z)

if __name__ == '__main__':
    import rostest
    rospy.init_node('tf_tree_connector_test', anonymous = True)
    rostest.rosrun('odometry_bench', 'tf_tree_connector_test', TfTreeConnectorTest)
