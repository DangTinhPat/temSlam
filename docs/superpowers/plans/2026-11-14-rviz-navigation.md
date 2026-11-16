# RViz navigation interaction

User explicitly requests RViz on the scanned room with standard pose/goal tools,
trajectory and costmap. This extends the existing navigation demo, not Nav2 or
sensor-based native motion. Keep original mesh and strict native grid intact.

- [x] Add ROS display/input adapter around NavigationGrid + Navigator. Consume
  PoseWithCovarianceStamped /rb_navigation/initialpose and PoseStamped
  /rb_navigation/goal_pose (map frame, finite planar unit quaternion). Invalid
  commands stop the current demo. Publish original mesh, static OccupancyGrid,
  inflated global costmap, local static crop, plan, executed trajectory, robot
  markers, map-to-base_link TF and explicit kinematic state. PointStamped stop.
- [x] Add RViz config with standard Interact, MoveCamera, SetInitialPose,
  SetGoal and PublishPoint tools; configure mesh/maps/paths/robot/TF topics.
  Launcher uses existing ROS conda environment, owns only its node and RViz,
  exits cleanly and never starts native simulator/velocity publisher.
- [x] Tests exercise message validation, pose reset/goal/cancel, arrival with
  final heading, serialized occupancy order/costs/crop origin. Run an actual
  ROS roundtrip through the same topics, verify RViz subscriptions and tools.
- [x] Open RViz with the actual mesh, demonstrate one goal, leave ready for user,
  document launch command and current limitation (kinematic, static costmap).

Verification: 64 tests pass. Actual ROS roundtrip cancels then reaches goal
within 2.45 cm and final yaw tolerance; RViz tools publish the configured topics.
Reviewer found shared mesh topic could overlay another viewer's map; changed to
/rb_navigation/mesh and restarted the owned RViz session. User goal events also
observed in RViz and adapter logs. GUI left open, no native backend/cmd_vel.
