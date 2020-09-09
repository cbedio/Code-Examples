# husky_sim_lidar_pan_behaviour

## Launch

### husky_sim_lidar_pan_behaviour.launch

#### Arguments

- robot_prefix - husky

#### husky_sim_lidar_pan_behaviour

##### parameters

- pan_speed - 1
- parent_link_name - /bottom_lidar_mount_link
- child_link_name - /top_lidar_mount_link
- max_angle - 1.569
- min_angle - -1.569
- frequency - 100
- lidar_cmd_topic_name - /husky_lidar_velocity_controller/command
