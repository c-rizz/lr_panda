#!/bin/bash

rostopic pub /panda_arm_effort_trajectory_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
    points:
    - positions:  [0, 0, 0, -1, 0, 1, 0]
      velocities: []
      accelerations: []
      effort:     [0, 0, 0, 0, 0, 0, 0]
      time_from_start: {secs: 10, nsecs: 0}
  path_tolerance:
  - {name: '', position: 0.0, velocity: 0.0, acceleration: 0.0}
  goal_tolerance:
  - {name: '', position: 0.0, velocity: 0.0, acceleration: 0.0}
  goal_time_tolerance: {secs: 0, nsecs: 0}" --once
