# docker

This submodule contains the Dockerfile used to build the project's [Docker image](https://hub.docker.com/repository/docker/joe3012/epsrc23/general).

This Dockerfile is quite basic, it does two main things. First, it installs ROS 2 Humble onto the Ubuntu 22 Cuda image. Then, it installs various requirements for the `vehicle-model` submodule including torch and jupyter.

## Building the Image

Building the image is quite simple, from the `epsrc-2023` root directory, run the following

```bash
cd epsrc-docker/v_1
docker build . -t joe3012/epsrc23:v_1
```

Be warned that this image can take over an hour to build, requires a large amount of download bandwidth and ends up being over 20GB in size.

## Updating the Image

To update the image, adhere to the following workflow:

1. make a new directory for the version, e.g. `v_2`
2. create the Dockerfile, use FROM and the previous version
3. use RUN and the commands that are required to update the image
4. use the command above to build the new image, tagging it appropriately and replacing the `v_1` directory with the one you just created