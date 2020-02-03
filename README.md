# Kalman Filter for Accurate State Estimation

Provides a ROS packages for creating accurate state estimation for a ROS-bot traversing in a line.

<img src="https://i.ibb.co/PthPgXv/Screen-Shot-2020-02-02-at-3-29-46-PM.png" width="350" />
<img src="https://i.ibb.co/yWPGDSF/Screen-Shot-2020-02-02-at-3-32-03-PM.png" width="350" />

# Files

Robot.py [pose|cmd_vel] : stores all the kalman filter and ROS node logic and publishes a node with type PoseWithCovarianceStamped from the kalman filter.

generate_graph.py : generates the graphs revealing the error and state estimation from the pose topic and the output of the kalman filters.

## Running Files

Robot.py [pose|cmd_vel]: requires a topic to obtain the linear velocity from (either pose or cmd_vel)

## Credits

John McCambridge - Dartmouth College '22
