# Overview 

This project attempts to explore model-based methods for vehicle control in driverless racing. This project is split into the following submodules:

* [epsrc-cluster-files](./cluster-files.md): Openshift cluster jobs and scripts for running simulations, generating racetracks and hosting a jupyter notebook.
* [epsrc-controller](./controller.md): A [ROS 2](https://docs.ros.org/en/humble/index.html) project containing code to autonomously navigate around a racetrack within the simulated environment provided by [EUFS Sim](https://gitlab.com/eufs/eufs_sim).
* [epsrc-docker](./docker.md): Dockerfiles for building this project's image. The image contains a ROS 2 install as well as the libraries used by the codebase. The image runs Ubuntu 22.
* `epsrc-eufs-msgs`: A [fork](https://gitlab.com/eufs/eufs_msgs) containing ROS 2 messages required by EUFS sim.
* `epsrc-eufs-sim`: A fork of EUFS Sim, containing some modifications. Namely, the ability to generate randomly shaped race tracks.
* [epsrc-sim-data-collection](./sim-data-collection.md): Tools for collecting simulation data, as well as running analyses, visualisations, integrity checks and vehicle model evaluations.
* `epsrc-ugrdv-msgs`: A fork of the UGRacing Driverless team's ROS 2 messages.
* [epsrc-vehicle-model](./vehicle-model.md): A python library and jupyter notebook containing a pytorch implementation of a neural network based vehicle model.

This project can run on the SOCS Openshift cluster. However, it can sometimes be more comfortable to work locally when making changes. For this to be possible, an Ubuntu 22 install is required. Please install [ROS 2](https://docs.ros.org/en/humble/index.html) and [Docker](https://docs.docker.com/) read through their documentations. It is especially important that you understand ROS 2's build system and [colcon](https://colcon.readthedocs.io/en/released/), their build tool.
