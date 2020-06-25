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
from geographic_msgs.msg import GeoPose
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import NavSatFix

from robot_localization.srv import SetDatum

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
    service set_datum callback.
    This method will collect currnt location's gps signal to set the DATUM.
    And reset the ekf_node initial pose. This means the datum will be the origin of map.
    """
    global DATUM

    SERVICE_INIT_STATE.call()

    lati_avg, long_avg = utility.get_avg_gps(topic=NAME_TOPIC_GPS)
    DATUM = (lati_avg, long_avg, 0.0)

    # -- Use robot_localization navsat_transform_node service to set datum.
    # -- Note: Changing existing parameter ~datum doesn't work

    geo_pose = GeoPose()
    geo_pose.position.latitude = DATUM[0]
    geo_pose.position.longitude = DATUM[1]
    geo_pose.position.altitude = DATUM[2]
    geo_pose.orientation.x = 0.0
    geo_pose.orientation.y = 0.0
    geo_pose.orientation.z = 0.0
    geo_pose.orientation.w = 1.0

    SERVICE_NAVSAT_DATUM(geo_pose)  # navsat_transform_node datum
    PUB.publish(geo_pose)

    rospy.loginfo("gps_nav: DATUM = {}, {}".format(DATUM[0], DATUM[1]))

    return EmptyResponse()

def _get_avg_gps(topic, count=10):
    """
    Get average gps signal (deault to 10)
    """
    subscriber = rospy.Subscriber(name=topic, data_class=NavSatFix, callback=_callback)

    rate = rospy.Rate(hz=1)
    while len(LATIS) < count+1:
        rate.sleep()

    subscriber.unregister()

    latt_avg = np.mean(LATIS)
    long_avg = np.mean(LONGS)

    global LATIS, LONGS
    LATIS = list()
    LONGS = list()

    rospy.loginfo("GPS {} signal average = {} {}".format(count, latt_avg, long_avg))
    return (latt_avg, long_avg)

if __name__ == "__main__":

    rospy.init_node(name='gps_nav', anonymous=False)

    # -- Get parameters
    """
    Based on http://docs.ros.org/melodic/api/robot_localization/html/navsat_transform_node.html
    If wait_for_datum is true, navsat_transform_node will wait for ~datum parameter or ~set_datum service.
    If it is false. it will use the first gps signal to be the datum.
    In my testing. The name of the service is just "~datum", not ~set_datum.
    """
    NAME_TOPIC_GPS = rospy.get_param(param_name="~top_gps")
    name_service_init_state = rospy.get_param(param_name="~srv_init_state")
    name_service_navsat_datum = rospy.get_param(param_name="~srv_datum")  # navsat service name for setup datum
    name_ns_move_base = rospy.get_param(param_name="~ns_move_base")

    # -- Get ServiceProxy, Action client
    SERVICE_INIT_STATE = rospy.ServiceProxy(name=name_service_init_state, service_class=Empty)
    SERVICE_NAVSAT_DATUM = rospy.ServiceProxy(name=name_service_navsat_datum, service_class=SetDatum)
    ACTION_MOVE_BASE = actionlib.SimpleActionClient(ns=name_ns_move_base, ActionSpec=MoveBaseAction)
    ACTION_MOVE_BASE.wait_for_server()

    # -- Node function
    PUB = rospy.Publisher(name="/datum", data_class=GeoPose, queue_size=1)

    # -- Set DATUM, make sure it is the same with navsat_transform_node datum
    # -- This DATUM means the gps values of the origin of map
    LATIS = list()
    LONGS = list()
    DATUM = list()
    cb_set_datum("")  # Important! this calling will stop the launch execute /fsm/start.

    # -- Node function
    service_goal = rospy.Service(name='~goal', service_class=GoalGPS, handler=cb_goal)
    service_datum = rospy.Service(name='~set_datum', service_class=Empty, handler=cb_set_datum)

    rospy.spin()
