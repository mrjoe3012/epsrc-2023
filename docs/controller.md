# controller

This project contains some autonomous racing control code. This controller is very basic and was written just to be able to complete simulation runs and collect data.

## Parameters

| Name | Default Value | Description |
| - | - | - |
| perception_model | realistic | The perception model profile passed on to the [simulated perception node](./sim-data-collection.md). |
| drive_request_topic | /drive_request | The ugrdv_msgs/DriveRequest topic which contains controller-issued drive commands along with metadata. |
| car_request_topic | /car_request | The ugrdv_msgs/CarRequest topic. Pre-PID controls containing target velocities. |
| ackermann_topic | /cmd | The ackermann_msgs/AckermannDriveStamped topic containing the command consumed by EUFS Sim. |
| gt_cones_topic | /ground_truth/track | The eufs_msgs/ConeArrayWithCovariance topic containing the ground truth cone positions and colours in the global frame. |
| perception_cones_topic | /perception_cones | ugrdv_msgs/Cone3dArray simulated perception cones. |
| perception_cones_vis_topic | /perception_cones/vis | visualization_msgs/MarkerArray message for visualising simulated perception in `rviz2`. |
| simulated_cones_topic | /simulated_cones | eufs_msgs/ConeArrayWithCovariance topic containing the simulated perception before it has been converted to the equivalent UGRDV message. |
| wheel_speeds_topic | /ros_can/wheel_speeds | eufs_msgs/WheelSpeedsStamped message containing the simulated wheel speeds from EUFS sim. |
| vcu_status_topic | /vcu_status | ugrdv_msgs/VCUStatus published by the controller, containing the current steering angle and wheel speeds. |
| gt_car_state_topic | /ground_truth/state | eufs_msgs/CarState message containing the grounf truth pose of the car. |
| path_planning_beam_width | 3 | The beam width for the beam search algorithm in the path planning pipeline. |
| sensor_fov | 180.0 | The simulated perception field-of-view, in degrees. |
| sensor_range | 12.0 | The simulated perception range, in metres. |