#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

from ros_gps_nav.srv import GoalGPS, GoalGPSResponse
from ros_gps_nav.srv import GetGPS, GetGPSResponse

import geonav_transform.geonav_conversions as gc
import utility

def callback_goal(request):
    """
    Callback for ~goal service
    """
    x, y = gc.ll2xy(request.latitude, request.longitude, DATUM[0], DATUM[1])

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1
    publisher.publish(goal)

    rospy.loginfo("GPS target = {} {}".format(request.latitude, request.longitude))
    rospy.loginfo("MAP target = {} {}".format(x, y))

    return GoalGPSResponse()

def callback_get_gps(request):
    """
    Callback for ~get_gps service
    """
    latt_avg, long_avg = utility.get_gps(topic=GPS_SENSOR_TOPIC_NAME)
    gps = GetGPSResponse()
    gps.latitude = latt_avg
    gps.longitude = long_avg
    return gps

def callback_reset_datum(request):
    """
    Callback for ~reset_datum service
    """
    # TODO: reset gps_nav datum when robot is not located at the origin.
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='gps_nav', anonymous=False)

    # -- Get parameter from launch
    GPS_SENSOR_TOPIC_NAME = rospy.get_param(param_name="~topic_gps")
    move_base_goal_topic_name = rospy.get_param(param_name="~topic_move_base_goal")
    navsat_datum_parameter_name = rospy.get_param(param_name="~param_datum")
    navsat_datum_service_name = rospy.get_param(param_name="~service_datum")

    """
    Based on http://docs.ros.org/melodic/api/robot_localization/html/navsat_transform_node.html
    If wait_for_datum is true, navsat_transform_node will wait for ~datum parameter or ~set_datum service.
    If it is false. it will use the first gps signal to be the datum.
    In my testing. The name of the service is just "~datum", not ~set_datum.
    """

    # -- Make sure gps_nav datum is the same with navsat_transform_node datum
    if rospy.has_param(param_name=navsat_datum_parameter_name):
        DATUM = rospy.get_param(param_name=navsat_datum_parameter_name)
    else:
        latt_avg, long_avg = utility.get_gps(topic=GPS_SENSOR_TOPIC_NAME)
        DATUM = (latt_avg, long_avg, 0.0)
        utility.set_navsat_datum(datum_service=navsat_datum_service_name, datum=DATUM)

    rospy.loginfo("gps_nav DATUM = {}, {}, {}".format(DATUM[0], DATUM[1], DATUM[2]))

    # -- Node function
    service_goal = rospy.Service(name='~goal', service_class=GoalGPS, handler=callback_goal)
    service_get_gps = rospy.Service(name='~get_gps', service_class=GetGPS, handler=callback_get_gps)
    service_datum = rospy.Service(name='~reset_datum', service_class=Empty, handler=callback_reset_datum)
    publisher = rospy.Publisher(name=move_base_goal_topic_name, data_class=PoseStamped, queue_size=1)

    rospy.spin()
