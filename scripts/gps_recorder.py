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
    global RECORDS
    latt_avg, long_avg = utility.get_avg_gps(topic=GPS_SENSOR_TOPIC_NAME, count=1)
    RECORDS.append((latt_avg, long_avg))
    return EmptyResponse()

def cb_save(request):
    with open(name=PATH_FILE, mode="w") as fileio:
        for i in RECORDS:
            fileio.write("{} {}\n".format(i[0], i[1]))
    return EmptyResponse()

def cb_load(request):
    global RECORDS
    RECORDS = list()
    with open(name=PATH_FILE, mode="r") as fileio:
        data = fileio.readlines()
    for line in data:
        RECORDS.append(line.split(sep=" "))
    return EmptyResponse()

def cb_clear(request):
    global RECORDS
    RECORDS = list()
    return EmptyResponse()

def cb_delete(request):
    if os.path.exists(PATH_FILE):
        os.remove(PATH_FILE)
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='gps_recorder', anonymous=False)
    RECORDS = list()

    # -- Get parameters
    GPS_SENSOR_TOPIC_NAME = rospy.get_param(param_name="~topic_gps", default="/gps_fix")
    path_folder = rospy.get_param(param_name="~path_folder")

    # -- Create path of file
    if not os.path.exists(path=path_folder):
        os.mkdir(path_folder)
    PATH_FILE = path_folder + "/records.txt"

    # -- Node function
    rospy.Service(name='~record', service_class=Empty, handler=cb_record)
    rospy.Service(name='~save', service_class=Empty, handler=cb_save)
    rospy.Service(name='~load', service_class=Empty, handler=cb_load)
    rospy.Service(name='~clear', service_class=Empty, handler=cb_clear)
    rospy.Service(name='~delete', service_class=Empty, handler=cb_delete)

    rospy.spin()
