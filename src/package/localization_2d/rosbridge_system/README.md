# rosbridge_system

Subscribed Topic:

  (1) task_switch （std_msgs/Header)
 
  (2) nav_ctrl_status (yocs_msgs/NavigationControlStatus)
 
  (3) amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
 
  (4) odom (nav_msgs/Odometry)
 
  (5) cmd_vel (geometry_msgs/Twist)
 
  (6) obstacle_avoidance/distance (std_msgs/Float32)
 
  (7) task_state (diagnostic_msgs/DiagnosticStatus)
 
  (8) move_base/result (move_base_msgs/MoveBaseActionResult)
 
  (9) battery (sensor_msgs/BatteryState)

  (10) nav_ctrl (yocs_msgs/NavigationControl)

  (11) move_base/TebLocalPlannerROS/teb_poses (geometry_msgs/PoseArray)

  (12) global_pose_2 (nav_msgs/Odometry)
  
  ------
 
 Published Topic:
 
  (1) music/median_control (std_msgs/Header)
  
  (2) music/median_player_server/cancel (actionlib_msgs/GoalID)
  
  (3) music/median_player_server/goal (median_player/MedianPlayActionGoal)
  
  (4) alert/median_control (std_msgs/Header)
  
  (5) alert/median_player_server/cancel (actionlib_msgs/GoalID)
  
  (6) alert/median_player_server/goal (median_player/MedianPlayActionGoal)
  
  (7) task_switch （std_msgs/Header)
  
  (8) task_state (diagnostic_msgs/DiagnosticStatus)
  
  (9) joy_start_0 (sensor_msgs/JoyFeedbackArray)
  
  (10) initialpose (geometry_msgs/PoseWithCovarianceStamped)
  
  (11) nav_ctrl (yocs_msgs/NavigationControl)
  
  (12) diagnostics (diagnostic_msgs/DiagnosticArray)

  (13) homing_control_server/cancel (actionlib_msgs/GoalID)
  
  ------
  
  Task State Feedback:
  
  (0)  auto_charge
  
  (1) cartographer_node 
 
  (2) amcl 
  
  (3) move_base 
  
  (4) shelf_detector 
  
  ------
  
  Alert Feedback:
  
  (1) finish auto charging  <- battery
  
  (2) obstacles nearby <- obstacle_avoidance/distance & shelf_detector
  
  (3) turn right <- odom / cmd_vel
  
  (4) turn left <- odom / cmd_vel
  
  (5) move_base goal reached <- move_base/result
  
  (6) battery charging <- auto_charge
  
  (7)
  
  (8)
  
  (9) low battery <-  battery
  
  (10) in operation <- nav_ctrl_status & odom / cmd_vel
  
  -----
  
  Rosparam:
  
  (1) rosbridge_system/cmd_vel_music : BOOL (False) > Whether to judge a turn by topic cmd_vel and enable turn alert (Default = False)
  
  (2) rosbridge_system/turn_threshold_omega : FLOAT (0.2) > Angle speed threshold that triggers turn alarm  (Default = 0.2)
  
  (3) rosbridge_system/turn_threshold_num: INT (10) > Angular Velocity Samples. Large values can cause long delays; Small value is prone to misjudgment. (Default = 10)
  
  (4) rosbridge_system/odom_vel_music : BOOL (True)> Whether to judge a turn by topic odom and enable turn alert (Default = True)
  
  (5) rosbridge_system/obstacle_alert_distance : FLOAT (1.0) > Obstacle distance that triggers alarm (Default = 1.0)
  
  (6) rosbridge_system/bgm_enabled : BOOL (True) > Whether to play background music (Default = True)
  
  (7) rosbridge_system/bgm_mode : STRING ('loop') > single/loop/random (Default = 'loop')
  
  (8) rosbridge_system/bgm_id : INT (10) > bgm file id (Default = 10)
  
  (9) rosbridge_system/low_battery: FLOAT (0.1) > Power that triggers the autocharge_on task (Default = 0.1)
  
  (10) rosbridge_system/full_battery : FLOAT (0.99) > Power that triggers the autocharge_off task after autocharging (Default = 0.99)
  
  (11) rosbridge_system/goal_reached_alert : BOOL (True) > Whether to enable goal reaching alert (Default = True)
  
  (12) rosbridge_system/obstacle_alert : BOOL (True)  > Whether to enable obstacle alert (Default = True)
  
  (13) rosbridge_system/charging_state_alert : BOOL (True)  > Whether to enable charging state alert (Default = True)

  (14) rosbridge_system/if_teb_turn_judge > Whether to predict turn behavior by TEB (Default = True)

  (15) rosbridge_system/teb_turn_yaw (Default = 0.4)

  (16) rosbridge_system/bgm_mode (Default = 'loop')

  (17) rosbridge_system/bgm_id (Default = 10)




  
  
 
