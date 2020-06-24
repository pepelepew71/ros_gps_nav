#!/usr/bin/env python

"""
GPS navigation node
This node should start with the robot_localization navsat_transform_node.
"""

from __future__ import print_function
from __future__ import division

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse

from ros_gps_nav.srv import GoalGPS, GoalGPSResponse

import geonav_transform.geonav_conversions as gc
import utility

def cb_goal(request):
    """
    Callback for ~goal service, using actionlib.SimpleActionClient
    """
    # -- east zero yaw
    # x, y = gc.ll2xy(request.latitude, request.longitude, DATUM[0], DATUM[1])

    # --north zero yaw
    x, y = gc.ll2xy_nzy(request.latitude, request.longitude, DATUM[0], DATUM[1])

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

def cb_set_datum(request):
    """
    Callback for ~set_datum service.
    This method will collect currnt location's gps signal to set the DATUM.
    This Datum should be the origin of map.
    """
    global DATUM
    lati_avg, long_avg = utility.get_avg_gps(topic=NAME_TOPIC_GPS)
    DATUM = (lati_avg, long_avg, 0.0)

    utility.set_navsat_datum(datum_service=NAME_SERVICE_NAVSAT_DATUM, datum=DATUM)

    rospy.loginfo("gps_nav DATUM = {}, {}, {}".format(DATUM[0], DATUM[1], DATUM[2]))
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
    NAME_TOPIC_GPS = rospy.get_param(param_name="~topic_gps", default="/gps_fix")
    NAME_SERVICE_NAVSAT_DATUM = rospy.get_param(param_name="~service_datum", default="/datum")  # navsat service name for setup datum
    name_ns_move_base = rospy.get_param(param_name="~ns_move_base", default="/move_base")

    # -- Set DATUM, make sure it is the same with navsat_transform_node datum
    # -- This DATUM means the gps values of the origin of map
    DATUM = list()
    cb_set_datum("")  # TODO, <<< important! this will stop the launch calling /fsm/start.
                      # This is what I want but I don't know why.

    # -- Get ServiceProxy, Action client
    ACTION_MOVE_BASE = actionlib.SimpleActionClient(ns=name_ns_move_base, ActionSpec=MoveBaseAction)
    ACTION_MOVE_BASE.wait_for_server()

    # -- Node function
    service_goal = rospy.Service(name='~goal', service_class=GoalGPS, handler=cb_goal)
    service_datum = rospy.Service(name='~set_datum', service_class=Empty, handler=cb_set_datum)

    rospy.spin()
