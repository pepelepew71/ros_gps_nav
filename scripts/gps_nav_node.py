#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
from geometry_msgs.msg import PoseStamped

from ros_gps_nav.srv import GoalGPS, GoalGPSResponse
from ros_gps_nav.srv import GetGPS, GetGPSResponse

import geonav_transform.geonav_conversions as gc
import utility


class NavigationNode(object):

    def __init__(self, topic_gps):
        self.topic_gps = topic_gps
        self.service_goal = rospy.Service(name='/gps_nav/goal', service_class=GoalGPS, handler=self._callback_goal)
        self.service_gps = rospy.Service(name='/gps_nav/get_gps', service_class=GetGPS, handler=self._callback_get_gps)
        self.publisher = rospy.Publisher(name='/move_base_simple/goal', data_class=PoseStamped, queue_size=1)

    def _callback_goal(self, request):
        if not rospy.has_param(param_name="/gps_nav/datum"):
            rospy.logerr("can't find parameter /gps_nav/datum")

        datum = rospy.get_param(param_name="/gps_nav/datum")
        datum = [float(i) for i in datum.split(" ")]
        x, y = gc.ll2xy(request.latitude, request.longitude, datum[0], datum[1])

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1
        self.publisher.publish(goal)

        rospy.loginfo("GPS target = {} {}".format(request.latitude, request.longitude))
        rospy.loginfo("MAP target = {} {}".format(x, y))

        return GoalGPSResponse()

    def _callback_get_gps(self, request):
        latt_avg, long_avg = utility.get_gps(topic=self.topic_gps)
        gps = GetGPSResponse()
        gps.latitude = latt_avg
        gps.longitude = long_avg
        return gps


if __name__ == "__main__":

    rospy.init_node(name='gps_nav_node', anonymous=False)

    topic_gps_raw = rospy.get_param(param_name="~topic_gps_raw")
    utility.set_gps_nav_datum(topic=topic_gps_raw)

    NavigationNode(topic_gps=topic_gps_raw)

    rospy.spin()
