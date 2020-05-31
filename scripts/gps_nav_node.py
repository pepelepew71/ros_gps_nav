#!/usr/bin/env python

"""
gps navigation node
"""

from __future__ import print_function
from __future__ import division

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse
from tf import TransformListener

from ros_gps_nav.srv import GoalGPS, GoalGPSResponse
from ros_gps_nav.srv import GetGPS, GetGPSResponse

import geonav_transform.geonav_conversions as gc
import utility

def cb_goal(request):
    """
    Callback for ~goal service, using actionlib.SimpleActionClient
    """
    x, y = gc.ll2xy(request.latitude, request.longitude, DATUM[0], DATUM[1])

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1
    rospy.loginfo("GPS target = {} {}".format(request.latitude, request.longitude))
    rospy.loginfo("MAP target = {} {}".format(x, y))
    state = ACTION_MOVE_BASE.send_goal_and_wait(goal=goal)

    response = GoalGPSResponse()
    response.state = state

    return response

def cb_get_gps(request):
    """
    Callback for ~get_gps service
    """
    latt_avg, long_avg = utility.get_gps(topic=GPS_SENSOR_TOPIC_NAME)
    gps = GetGPSResponse()
    gps.latitude = latt_avg
    gps.longitude = long_avg
    return gps

def cb_reset_datum(request):
    """
    Callback for ~reset_datum service
    """
    # TODO: reset gps_nav datum when robot is not located at the origin.
    global DATUM
    latt_avg, long_avg = utility.get_gps(topic=GPS_SENSOR_TOPIC_NAME)
    rospy.loginfo("Current GPS = {} {}".format(latt_avg, long_avg))
    DATUM = (latt_avg, long_avg, 0.0)
    utility.set_navsat_datum(datum_service=NAVSAT_DATUM_SERVICE_NAME, datum=DATUM)
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='gps_nav', anonymous=False)

    # -- Get parameters
    """
    Based on http://docs.ros.org/melodic/api/robot_localization/html/navsat_transform_node.html
    If wait_for_datum is true, navsat_transform_node will wait for ~datum parameter or ~set_datum service.
    If it is false. it will use the first gps signal to be the datum.
    In my testing. The name of the service is just "~datum", not ~set_datum.
    """
    GPS_SENSOR_TOPIC_NAME = rospy.get_param(param_name="~topic_gps", default="/gps_fix")
    navsat_datum_parameter_name = rospy.get_param(param_name="~param_datum", default="/navsat/datum")
    NAVSAT_DATUM_SERVICE_NAME = rospy.get_param(param_name="~service_datum", default="/datum")
    ns_move_base = rospy.get_param(param_name="~ns_move_base", default="/move_base")
    TF_BASE_LINK = rospy.get_param(param_name="~tf_base_link", default="/base_link")

    # -- Set DATUM, make sure it is the same with navsat_transform_node datum
    # -- This DATUM means the gps values of the origin of map
    DATUM = list()

    if rospy.has_param(param_name=navsat_datum_parameter_name):
        DATUM = rospy.get_param(param_name=navsat_datum_parameter_name)
    else:
        latt_avg, long_avg = utility.get_gps(topic=GPS_SENSOR_TOPIC_NAME)
        DATUM = (latt_avg, long_avg, 0.0)
        utility.set_navsat_datum(datum_service=NAVSAT_DATUM_SERVICE_NAME, datum=DATUM)

    rospy.loginfo("gps_nav DATUM = {}, {}, {}".format(DATUM[0], DATUM[1], DATUM[2]))

    # -- Action client for move_base
    ACTION_MOVE_BASE = actionlib.SimpleActionClient(ns=ns_move_base, ActionSpec=MoveBaseAction)
    ACTION_MOVE_BASE.wait_for_server()
    TF_LISTENER = tf.TransformListener()

    # -- Node function
    service_goal = rospy.Service(name='~goal', service_class=GoalGPS, handler=cb_goal)
    service_gps = rospy.Service(name='~get_gps', service_class=GetGPS, handler=cb_get_gps)
    service_datum = rospy.Service(name='~reset_datum', service_class=Empty, handler=cb_reset_datum)

    rospy.spin()
