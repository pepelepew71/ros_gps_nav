#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPose

LATIS = list()
LONGS = list()

def get_avg_gps(topic, count=10):
    """
    Get average gps signal (deault to 10)
    """
    global LATIS, LONGS
    subscriber = rospy.Subscriber(name=topic, data_class=NavSatFix, callback=_callback)

    rate = rospy.Rate(hz=1)
    while len(LATIS) < count+1:
        rate.sleep()
    subscriber.unregister()

    latt_avg = np.mean(LATIS)
    long_avg = np.mean(LONGS)
    LATIS = list()
    LONGS = list()

    rospy.loginfo("GPS {} signal average = {} {}".format(count, latt_avg, long_avg))
    return (latt_avg, long_avg)

def _callback(msg_navsatfix):
    """
    Subroutine for get_gps(). As callback for gps sensor subscriber
    """
    global LATIS, LONGS
    rospy.loginfo("GPS = {} {}".format(msg_navsatfix.latitude, msg_navsatfix.longitude))
    LATIS.append(msg_navsatfix.latitude)
    LONGS.append(msg_navsatfix.longitude)
