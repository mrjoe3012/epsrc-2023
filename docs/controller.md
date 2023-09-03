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

## How it Works

The racing controller pipeline works by responding to each observation with a driving command. An observation is a set of 2 or more cones' positions and colours.

Upon receiving an observation, the controller constructs a network representation of the cones on the track. It does this by using [delaunay triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation), and constructing a graph defined by the resulting simplices. This graph, describing how cones are connected on the track, is not directly useful as we do not want to drive into cones, but in between pairs of them. So, we get the line graph of this network, connecting the centrepoints between pairs of cones if any of their original vertices are connected in the original graph.

This new graph representation now expresses the connections between all drivable points and maintains pointers to the original vertices used to construct this graph. Now, we define a cost function mapping from a path to a floating point number, taking into account the colours of cones that are being driven through and the variability of the track shape along the path. This cost function is adapted from [this paper](https://arxiv.org/abs/1905.05150). 

With the cost function in place, all that is left to do is to find a good path candidate. This is done using a beam search algorithm, with a fixed beam width and iteration count. Each drivable point accessible from the car's starting position is used as a starting node for the searching algorithm. A drivable point's connectivity to the car's starting position can be determined by referring to the original delaunay triangulation, as this is performed with the car's position added to the observation points.

The first drivable point in the best path candidate is used as the next target point. We define two additional functions, one mapping from the best path's cost to a speed multiplier, and another mapping from the target points' required steering angle to a speed multiplier. This allows us to drive more conservatively when we receive poor perception data or have to steer severely. The kinematic bicycle model is inverted and applied to determine the target steering angle. A base velocity is multiplied by both multipliers to determine the final target velocity.

Finally, the desired velocity goes through a [PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) controller, which attempts to achieve the target velocity using simulated wheel speeds from EUFS Sim. The final steering angle request and acceleration commands are sent to the simulator via ROS 2.

## Running the Controller

In order for the controller to do anything, EUFS Sim must also be started. The instructions below will launch both.

First, open two terminal windows.

Navigate into the repository's root directory. In each, type the following to build the code and initialise the environment.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Now, type the following into one of the terminals to launch the simulator:

```bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

In the launcher dialogue, tick the 'Publish Ground Truth' checkbox and then click 'Launch!'. This should open two windows, a mission control GUI and an 'rviz' window. The rviz window will display the car and cones on the track. If the rviz window isn't displaying anything, try changing the 'fixed frame' to 'base_footprint' in the top-left. Next, in the mission control window, click on 'Manual Driving', this is required for the simulator to respond to commands from the controller.

In the other terminal window, type the following to start the controller

```bash
ros2 launch epsrc_controller epsrc_controller.launch.py
```

The car should now start driving around the track. The mission control window has buttons for resetting the simulation is necessary.

The simulation can be stopped by closing both terminals, or using Ctr-C to interrupt the commands. In some cases, Ctr-C needs to be pressed more than once to terminate the simulator.
