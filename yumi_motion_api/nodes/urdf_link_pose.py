#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from yumi_motion_api.srv import UrdfLinkPose, UrdfLinkPoseResponse

class LinkPoseService:
    def __init__(self):
        rospy.init_node('link_pose_service')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.service = rospy.Service('get_urdf_link_pose', UrdfLinkPose, self.handle_get_link_pose)
        rospy.loginfo("Link pose service is ready.")

    def handle_get_link_pose(self, req):
        link_name = req.link_name
        try:
            transform = self.tf_buffer.lookup_transform('modul4r_station_base_link', link_name, rospy.Time(0))
            rospy.loginfo(transform)
            return UrdfLinkPoseResponse(transform)
        except tf2_ros.LookupException as e:
            rospy.logwarn("Could not get transform for link %s: %s", link_name, e)
            return UrdfLinkPoseResponse()

if __name__ == '__main__':
    try:
        LinkPoseService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
