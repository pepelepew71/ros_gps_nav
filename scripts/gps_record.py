#!/usr/bin/env python

"""
GPS recorder node.
This node provides services for recording gps waypoint, saving history to a file or reading from it.
"""

from __future__ import print_function
from __future__ import division

import os

import rospy
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Empty, EmptyResponse

import utility

def cb_record(request):
    lati_avg, long_avg = utility.get_avg_gps(topic=GPS_SENSOR_TOPIC_NAME, count=10)
    value = "{0},{1},{2}\n".format(lati_avg, long_avg, 1)
    rospy.set_param(param_name=PARAM_NAME_NAV_SETUP, param_value=value)
    return EmptyResponse()

def cb_save(request):
    csv_txt = rospy.get_param(param_name=PARAM_NAME_NAV_SETUP)
    with open(name=PATH_FILE, mode="w") as fileio:
        fileio.write(csv_txt)
    return EmptyResponse()

def cb_load(request):

    if os.path.exists(path=PATH_FILE):
        with open(name=PATH_FILE, mode="r") as fileio:
            csv_txt = fileio.read()
    else:
        csv_txt = ""
        rospy.loginfo("/gps_record: nav_setup.csv is not exist")

    rospy.set_param(param_name=PARAM_NAME_NAV_SETUP, param_value=csv_txt)

    return EmptyResponse()

def cb_clear(request):
    csv_txt = ""
    rospy.set_param(param_name=PARAM_NAME_NAV_SETUP, param_value=csv_txt)
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='gps_record', anonymous=False)

    # -- Get parameters
    GPS_SENSOR_TOPIC_NAME = rospy.get_param(param_name="~topic_gps", default="/gps_fix")
    PARAM_NAME_NAV_SETUP = rospy.get_param(param_name="~param_nav_setup", default="/nav_setup")
    PATH_FILE = rospy.get_param(param_name="~path_file")

    # -- Node function
    rospy.Service(name='~record', service_class=Empty, handler=cb_record)
    rospy.Service(name='~save', service_class=Empty, handler=cb_save)
    rospy.Service(name='~load', service_class=Empty, handler=cb_load)
    rospy.Service(name='~clear', service_class=Empty, handler=cb_clear)

    # -- Load nav_setup.csv
    cb_load("")

    rospy.spin()
