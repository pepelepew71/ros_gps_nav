#!/usr/bin/env python

"""
GPS recorder node.
This node provides services for recording gps waypoint, saving history to a file or reading from it.
"""

from __future__ import print_function
from __future__ import division

import os
import time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

from ros_gps_nav.srv import GetNav, GetNavResponse
from ros_gps_nav.srv import SetNav, SetNavResponse

import utility

def cb_record(request):

    if IS_PUB_STATUS:
        PUB_STATUS.publish("Recording")

    csvtxt = rospy.get_param(param_name=PARAM_NAME_NAV_SETUP)
    lati_avg, long_avg = utility.get_avg_gps(topic=GPS_SENSOR_TOPIC_NAME, count=10)
    csvtxt += "{0},{1},{2}\n".format(lati_avg, long_avg, 1)  # default to num 1
    rospy.set_param(param_name=PARAM_NAME_NAV_SETUP, param_value=csvtxt)

    if IS_PUB_STATUS:
        PUB_STATUS.publish("Done")
        time.sleep(1)
        PUB_STATUS.publish("")

    return EmptyResponse()

def cb_save(request):
    rospy.loginfo("/gps_rec: saving nav_setup.csv")
    csvtxt = rospy.get_param(param_name=PARAM_NAME_NAV_SETUP)
    with open(name=PATH_FILE, mode="w") as fileio:
        fileio.write(csvtxt)
    return EmptyResponse()

def cb_read(request):
    csvtxt = ""
    if os.path.exists(path=PATH_FILE):
        with open(name=PATH_FILE, mode="r") as fileio:
            lines = fileio.readlines()
        for line in lines:
            txt = line.rstrip()  # remove newline
            if txt:
                csvtxt += txt + "\n"
    else:
        rospy.loginfo("/gps_rec: nav_setup.csv is not exist")
    rospy.set_param(param_name=PARAM_NAME_NAV_SETUP, param_value=csvtxt)
    return EmptyResponse()

def cb_get(request):
    csvtxt = rospy.get_param(param_name=PARAM_NAME_NAV_SETUP)
    response = GetNavResponse()
    response.csvtxt = csvtxt
    return response

def cb_set(request):
    rospy.set_param(param_name=PARAM_NAME_NAV_SETUP, param_value=request.csvtxt)
    return SetNavResponse()

if __name__ == "__main__":

    rospy.init_node(name='gps_rec', anonymous=False)

    # -- Get parameters
    GPS_SENSOR_TOPIC_NAME = rospy.get_param(param_name="~topic_gps", default="/gps_fix")
    PARAM_NAME_NAV_SETUP = rospy.get_param(param_name="~param_nav_setup", default="/nav_setup")
    PATH_FILE = rospy.get_param(param_name="~path_file")
    IS_PUB_STATUS = rospy.get_param(param_name="~is_pub_status")

    # -- Node function
    if IS_PUB_STATUS:
        PUB_STATUS = rospy.Publisher(name="/status_for_web", data_class=String, queue_size=1)

    rospy.Service(name='~record', service_class=Empty, handler=cb_record)
    rospy.Service(name='~save', service_class=Empty, handler=cb_save)
    rospy.Service(name='~read', service_class=Empty, handler=cb_read)
    rospy.Service(name="~get", service_class=GetNav, handler=cb_get)
    rospy.Service(name="~set", service_class=SetNav, handler=cb_set)

    # -- Load nav_setup.csv
    cb_read("")

    rospy.spin()
