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

def get_avg_gps(topic, count=10):
    """
    Get average gps signal (deault to 10)
    """
    global LATTS, LONGS
    subscriber = rospy.Subscriber(name=topic, data_class=NavSatFix, callback=_callback)

    rate = rospy.Rate(hz=1)
    while len(LATTS) < count+1:
        rate.sleep()
    subscriber.unregister()

    latt_avg = np.mean(LATTS)
    long_avg = np.mean(LONGS)
    LATTS = list()
    LONGS = list()

    rospy.loginfo("GPS {} signal average = {} {}".format(count, latt_avg, long_avg))
    return (latt_avg, long_avg)

def _callback(msg_navsatfix):
    """
    Subroutine for get_gps(). As callback for gps sensor subscriber
    """
    global LATTS, LONGS
    rospy.loginfo("GPS = {} {}".format(msg_navsatfix.latitude, msg_navsatfix.longitude))
    LATTS.append(msg_navsatfix.latitude)
    LONGS.append(msg_navsatfix.longitude)

def set_navsat_datum(datum_service, datum):
    """
    Use robot_localization navsat_transform_node service to set datum.
    Note: Changing existing parameter ~datum will not effect navsat_transform_node
    """
    rospy.wait_for_service(service=datum_service)
    service = rospy.ServiceProxy(name=datum_service, service_class=SetDatum)
    geo_pose = GeoPose()
    geo_pose.position.latitude = datum[0]
    geo_pose.position.longitude = datum[1]
    geo_pose.position.altitude = datum[2]
    geo_pose.orientation.x = 0.0
    geo_pose.orientation.y = 0.0
    geo_pose.orientation.z = 0.0
    geo_pose.orientation.w = 1.0
    service(geo_pose)  # navsat_transform_node datum
