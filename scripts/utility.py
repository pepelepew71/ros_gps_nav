#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPose

from robot_localization.srv import SetDatum

LATTS = list()
LONGS = list()

def get_gps(topic, count=10):
    subscriber = rospy.Subscriber(name=topic, data_class=NavSatFix, callback=_callback)
    rate = rospy.Rate(hz=1)
    while len(LATTS) < count+1:
        rate.sleep()
    subscriber.unregister()

    latt_avg = np.mean(LATTS)
    long_avg = np.mean(LONGS)

    global LATTS, LONGS
    LATTS = list()
    LONGS = list()

    rospy.loginfo("GPS {} signal average = {} {}".format(count, latt_avg, long_avg))
    return (latt_avg, long_avg)

def _callback(msg_navsatfix):
    rospy.loginfo("GPS = {} {}".format(msg_navsatfix.latitude, msg_navsatfix.longitude))
    LATTS.append(msg_navsatfix.latitude)
    LONGS.append(msg_navsatfix.longitude)

def set_gps_nav_datum(topic):
    rospy.wait_for_service(service='/datum')
    service_datum = rospy.ServiceProxy(name='/datum', service_class=SetDatum)

    latt_avg, long_avg = get_gps(topic)

    datum = GeoPose()
    datum.position.latitude = latt_avg
    datum.position.longitude = long_avg
    datum.position.altitude = 0.0
    datum.orientation.x = 0.0
    datum.orientation.y = 0.0
    datum.orientation.z = 0.0
    datum.orientation.w = 1.0

    service_datum(datum)
    rospy.set_param(param_name="/gps_nav/datum", param_value="{} {}".format(latt_avg, long_avg))  # for navigation
