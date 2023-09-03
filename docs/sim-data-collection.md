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

## data_collector

This component collects simulation data. This tool doesn't need to be run by hand, the `epsrc-cluster-files` module contains scripts to run this alongside a headless EUFS Sim instance on the cluster.

This component works by looking at message metadata. Each mesasage published by the controller contains a 'meta' section, this section includes a unique message hash and an optional list of other hashes indicating which messages are associated to this one. This has been done to ensure no messages are lost as ROS 2 messages are delivered on a best-effort basis by default.

The data collector listens to the following messages.

| Topic | Message Type |
| - | - |
| /drive_request | ugrdv_msgs/DriveRequest |
| /car_request | ugrdv_msgs/CarRequest |
| /perception_cones | ugrdv_msgs/Cone3dArray |
| /vcu_status | ugrdv_msgs/VCUStatus |
| /ground_truth/cones | eufs_msgs/ConeArrayWithCovariance |
| /ground_truth/state | eufs_msgs/CarState |

Once these messages are received, they are place into an sqlite3 database. Upon terminating the data collection, the system will look for messages which are missing any dependencies, removing them from the database with a warning message.

## analysis - integrity_check

This is a small executable used within the simulation job to ensure that the simulation run went as expected. This executable returns non-zero if there are any signs of corruption in the database.

Usage (from the epsrc-2023 root directory):

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run sim_data_collection integrity_check <database1> <database2> ...
```

## analysis

This a larger suite of tools for interacting with simulation data.

There are a set of verbs that can be used to alter the behaviour of this program:

| Verb | Description |
| - | - |
| analyse | Iterates through a simulation dataset, collecting statistics on track violations and distance completed. |
| visualise | Plays back a simulation run with an elevated timescale. Also visualises a Kinematic Bicycle vehicle model's predictions of the car pose using the recorded vehicle commands. Optionally accepts the path to a saved pytorch model and visualises its predictions using recorded vehicle commands too. |
| plot | Uses a .json file generated by the 'analyse' verb to create visualisations of the data. |
| evaluate | Performs a quantitative evaluation of a neural network based vehicle model defined by a .pt file against a Kinematic Bicycle model on a set of simulation runs. |


### Usage

Each of these examples assume you are in the 'epsrc-2023' root directory and have set up the environment by running the following commands:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

analyse

```bash
ros2 run sim_data_collection analysis analyse <database1> <database2> ...
```

plot

```bash
ros2 run sim_data_collection analysis plot <input json>
```

visualise

```bash
ros2 run sim_data_collection analysis <neural network .pt file (optional)> <database 1> <database 2> ...
```

evaluate

```bash
ros2 run sim_data_collection analysis <neural network .pt file> <database 1> <database 2> ...
```