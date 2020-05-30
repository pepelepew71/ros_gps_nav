# ros_gps_nav

Based on [geonav_transform](http://wiki.ros.org/geonav_transform) .

Use navsat_transform_node datum as the origin of map. And transfer gps target to map by tf utm -> map.

Make sure gps_nav datum is the same with navsat_transform_node datum.

## 1. Node

gps_nav

## 2. Parameters

~topic_gps

(sensor_msgs/NavSatFix) gps sensor topic.

~param_datum

navsat_transform_node datum parameter

~service_datum

navsat_transform_node set_datum service

~ns_move_base

move_base namespace

## 3. Published Topics

move_base/action_topics

## 4. Services

~goal

(ros_gps_nav.srv/GoalGPS) Covert GPS target to UTM coordinate and send to /move_base_simple/goal

~get_gps

(std_srvs/Empty) Get current location's average GPS

## 5. Launch

```bash
roslaunch gps_navigation gps_nav.launch
```
