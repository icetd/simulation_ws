## simulation_ws
simple ros nav simulation

## Use
```
git clone https://github.com/icetd/simulation_ws.git
cd simualtion_ws
catkin_make_isolateda
source devel_isolated/setup.bash 
roslaunch simulation nav_map.launch
```

## update
- add [segment_global_planner](https://github.com/WLwind/segment_global_planner)

- /move_base/SegmentGlobalPlanner/base_global_planner
The real global planner implementation to ganerate path segments.You can feel free to set any general global planner plugins like global_planner/GlobalPlanner. If you don't set this parameter then line segement generator will be implemented and the performance is just like the previous version of segment_global_planner.
- /move_base/SegmentGlobalPlanner/threshold_point_on_line
Threshold that robot is considered on the tracking line within.(dynamic_reconfigure) If this value is too low, the robot may often derail and make a plan directly to your final goal.
/move_base/LineSegment/point_interval
The max interval of 2 points on a segment.(only for LineSegment)
- /move_base/SegmentGlobalPlanner/child_goal_threshold
Threshold that robot is considered reaching the goal within.(dynamic_reconfigure) You should make this larger than the tolerance of local planner and set a not too low frequency for global planner (5.0Hz is adequate for a small robot like Turtlebot3). If this value or the frequency of global planner is too low, the robot may accidentally finish the navigation action at a child goal.