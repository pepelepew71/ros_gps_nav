# ros_gps_nav

Based on [geonav_transform](http://wiki.ros.org/geonav_transform) .

When starting gps_nav_node. It will collect 10 GPS singals for setting navsat_transform_node's datum. All the GPS targets will be calulated based on this datum.

## 1. Node

gps_nav

## 2. Parameters

### ~topic_gps

(sensor_msgs/NavSatFix)

Topic name of gps sensor

### ~has_datum

(sensor_msgs/Bool, default: false)

## 3. Published Topics

### move_base_simple/goal

(geometry_msgs/PoseStamped)

## 4. Services

### gps_nav/goal

(ros_gps_nav.srv/GoalGPS)

Covert GPS target to UTM coordinate and send to /move_base_simple/goal

### gps_nav/get_gps

(std_srvs/Empty)

Get current location's average GPS

## 5. Launch

```bash
roslaunch gps_navigation gps_nav.launch
```
