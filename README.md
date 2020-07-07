# ros_gps_nav

Based on [geonav_transform](http://wiki.ros.org/geonav_transform) .

Use navsat_transform_node datum as the origin of map. Then transfer gps target to map by tf utm -> map.

Make sure gps_nav datum is the same with navsat_transform_node datum.

## Node

### 1. gps_nav

#### 1.1. Parameters

```~top_gps``` : gps sensor topic.

```~srv_init_state``` : From ros_my_localization/init_state.py. For setting initial pose in ekf.

```~srv_datum``` : navsat_transform_node's set_datum service

```~ns_move_base``` : move_base's namespace

#### 1.2. Published Topics

move_base/action_topics

#### 1.3. Services

```~goal``` (ros_gps_nav.srv/GoalGPS) : Covert GPS target to UTM coordinate and send to /move_base_simple/goal

```~set_datum``` (std_srvs/Empty) : Set current location's GPS signal as the navsat_transform_node datum. It will be the origin of map when using ekf node.

---

### 2. gps_rec

#### 2.1. Parameters

```~topic_gps```

```~param_nav_setup```

```~path_file```

#### 2.2. Services

```~record```

```~save```

```~read```

```~get```

```~set```
