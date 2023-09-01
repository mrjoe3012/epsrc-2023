# sim-data-collection

## perception_model

This executable is used by the data collection pipeline. It works by receiving ground truth cone colours and positions from the simulator, distorting them according to a probabilistic perception model and publishing them back into the ROS 2 ecosystem for use by the controller.

Once ground truth messages are received, four types of distortions are applied. The first is adding noise to each position coordinate. This noise is zero-mean Gaussian with standard deviation defined as a function of distance based on the model parameters. The second is mis-reporting of cone colours, this is done based using a single probability value per detected cone. The third is missing a cone entirely, which is once again decided by a single probability value. And finally, there is the possibility of hallucinating cones, which is decided similarly to the previous two.

Model parameters can be tweaked. This can be done by modifying the json files in `epsrc-sim-data-collection/models` and then rebuilding the project.

### Parameters

| Name | Default Value | Description |
| - | - | - |
| gt_car_state_topic | /ground_truth/state | Topic to listen for eufs_msgs/CarState messages containing the ground truth car pose. |
| gt_cones_topic | /ground_truth/track | eufs_msgs/ConeArrayWithCovariance topic containing the ground truth colours and positions of cones in the global frame. |
| perception_model | realistic | One of 'realistic', 'good' or 'poor'. This controls the quality of the perception simulation. |
| sensor_fov | 110.0 | The field-of-view of the simulated perception, in degrees. |
| sensor_range | 12.0 | The effective range of the simulated perception, in metres. |
| simulated_cones_topic | /ugrdv/perception/epsrc_cones | eufs_msgs/ConeArrayWithCovariance topic containing the simulated cone colours and positions. |

### Example

Below is an example on building, running and setting parameters. Run the following commands from the `epsrc-2023` repository root directory.

```bash
colcon build
source install/setup.bash
ros2 run sim_data_collection perception_model --ros-args \
    -p sensor_range:=15.0 \
    -p sensor_fov:=180.0 \
    -p perception_model:=poor
```

### Visualising the Cones

The perception cones can be visualised using a built-in ROS 2 tool call `rviz2`. Below is an example on running the simulation and controller, and visualising the simulated perception output.

Open two terminals, do the following in each:

First, cd into the `epsrc-2023` root directory. Then run

```bash
colcon build
source install/setup.bash
```

Now, in one of the temrinals,

To start the simulation:

```bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

Tick the 'Publish Ground Truth' checkbox, and then click 'Launch!'.

In another terminal,

To start the controller and simulated perception:

```bash
ros2 launch epsrc_controller epsrc_controller.launch.py
```

After launching EUFS Sim, an `rviz2` window should pop up. Type 'base_footprint' into the box labelled 'Fixed Frame' on the left hand side. Then click on 'Add', 'By topic', 'MarkerArray' under '/perception_cones/vis'. This should now display the distorted perception cones. By default, the '/fusion/cones' topic should be visible. This is EUFS Sim's default perception, which is near ground truth.
