# Usage Guide

This document will describe the steps required to run the full vehicle model training and evaluation workflow. An overview of the workflow is shown below.

1. First-time setup - 30 minutes
2. Track Generation - 1 hour
3. Running Simulations - 8 hours
4. Data Analysis - 1 hour
5. Notebook Setup - 10 minutes
6. Simulation Data Preprocessing - 3 hours
7. Neural Network Training

## First-time Setup

In order to be able to run the code and cluster jobs, some files need to be setup on the cluster.

1. SSH into the cluster

    ```
    ssh <your-id>@idagpu-head.dcs.gla.ac.uk
    ```

2. In your home directory, there should be a directory whose name ends in 'vol1claim', CD into this directory.

    ```
    cd <your-id>vol1claim
    ```

3. Recursively clone the project's repository

    ```
    git clone --recursive https://github.com/mrjoe3012/epsrc-2023
    ```

4. Allow access to jobs running on the cluster

    ```
    chown -R nfsnobody ./
    ```

5. Start up the `ssh-container`. You can do this by copy-pasting the job file contents into the create job dialogue on the openshift web console. Make sure you replace '2663460avol1claim' with the name of your own persistent volume claim and '2663460aproject' with the name of your own openshift cluster project.

6. Click on `pods`, and then select the pod associated with the `ssh-container` job. Select 'terminal' and type the following commands:

    ```
    cd /nfs/epsrc-2023
    source /opt/ros/humble/setup.bash
    colcon build
    ```

7. All builds should finish without errors

## Track Generation

1. Startup or use an existing `ssh-container` job.

2. Enter the `ssh-container` pod's terminal

3. Initialise the shared seed file with the following commands

    ```
    cd /nfs
    touch seed.txt
    /nfs/epsrc-2023/epsrc-cluster-files/scripts/next_value.py -r seed.txt
    ```

4. Configure the track generation job

    | Variable | Description |
    | - | - |
    | 'parallelism' | Should be set to 100 or less to prevent slowing down the cluster. Lower numbers of parallel jobs result in slower completion time for the job. |
    | 'completions' | Each completion generates at least 10 tracks, so if completions is set to 1000, expect to get at least 10,000 tracks. |

5. Copy paste the contents of `epsrc-2023/epsrc-cluster-files/jobs/track_generation.yml` into the create job dialogue. Make sure you replace '2663460avol1claim' with the name of your own persistent volume claim and '2663460aproject' with the name of your own openshift cluster project.

6. This job should complete without failures. The newly generated tracks can be found in `~/<id>vol1claim/epsrc-2023/install/eufs_tracks/share/eufs_tracks`. The 'images' folder contains top-down depictions of the newly generated tracks.

## Running Simulations

1. Start-up a new, or use an existing instance of the `ssh-container` job.

2. Enter into the `ssh-container` pod's terminal.

3. Set up the shared track file with the following command

    ```
    cd /nfs
    touch track.txt
    /nfs/epsrc-2023/epsrc-cluster-files/scripts/next_value.py -r track.txt
    ```

4. Copy-paste the contents of the `epsrc-2023/epsrc-cluster-files/jobs/run_simulations.yml` job into the create job dialogue. Make sure you replace '2663460avol1claim' with the name of your own persistent volume claim and '2663460aproject' with the name of your own openshift cluster project.

5. Configure the job

    | Variable | Description |
    | - | - |
    | `parallelism` | This should be set to 100 or below to prevent slowing down the cluster. |
    | `completions` | This controls how many simulations are run. This should always be set to 10-20% less than the number of available racetracks as jobs which fail due to crashes or corruptions will need to be re-run and will use a new track.

6. This job may have some failed pods, this is due to some bugs in the simulator or occasional corruption of simulation data, these failures do not affect the completion of the job overall. After around 8 hours all simulations should complete.

7. It is a good idea to move the simulation data to another directory. For example, you can execute the commands below to create a new directory and move the simulation data there. Run this from the `ssh-container` pod's terminal. These commands will take a few minutes to execute.

    ```
    mkdir /nfs/sim_data
    mv /nfs/epsrc-2023/install/sim_data_collection/share/sim_data_collection/*db3 /nfs/sim_data
    ```

## Data Analysis

1. Start up a new, or use an existing instance of the `ssh-container` job.

2. Enter into the `ssh-container` pod's terminal.

3. CD to the persistent volume claim directory

    ```
    cd /nfs
    ```

4. If this is not the first time you have ran this job, beware that you must first delete or move the 'analysis.json' file under the `/nfs` directory. If you do not do this, the job's pods will append their results to the existing 'analysis.json' file and the data will not make any sense. To delete the file, run the following command:

    ```
    rm analysis.json
    ```

5. Set up the shared process number and output file with the following commands

    ```
    cd /nfs
    touch analysis.json proc_num.txt
    /nfs/epsrc-2023/epsrc-cluster-files/scripts/next_value.py -r proc_num.txt
    ```

6. Copy-paste the contents of the `/nfs/epsrc-2023/epsrc-cluster-files/jobs/analyse_data.yml` into the create job dialogue. Make sure you replace '2663460avol1claim' with the name of your own persistent volume claim, '2663460aproject' with the name of your own openshift cluster project and '/nfs/epsrc-2023/install/sim_data_collection/share/sim_data_collection/*.db3' with the path to the simulation dataset to be analysed if it has been moved from the default directory.

7. Now run the job. Each pod instance will attempt to process 10 databases. So if you have 10,000 simulation datasets, you will need to set the 'completions' variables to 1000.

8. This job should complete without failures. Once complete, all of the resulting data will be found in `/nfs/analysis.json`. Download or move this file so that it can be visualised.

9. To make plots for the data, enter into the `ssh-container` pod's terminal and run the following commands:

    ```
    cd /nfs/epsrc-2023
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run sim_data_collection analysis plot /nfs/analysis.json
    ```

10. This will create new plots in the working directory which can be downloaded and viewed.

11. To visualise individual simulations, please refer to the [sim-data-collection](./sim-data-collection.md) page. Keep in mind that this task required you to run the scripts locally instead of on the cluster, as the visualisation attempts to create a window on the host machine.

## Notebook Setup

1. Training the neural network requires use of a jupyter notebook. This notebook can be set up on the cluster so that it can be accessed via an internet browser. Firstly, go to the cluster console page and login.

2. Now, click on 'networking', 'services' and then 'create service'. Then copy-pase [this configuration](https://git.dcs.gla.ac.uk/RichardMccreadie/openshift-applications/-/blob/master/tutorial/pythonJupyterNotebookService.yaml) into the dialogue. Be sure to change 'richardstestproject' to the name of your own openshift cluster project. When you are happy with the configuration, click 'create'.

3. Click on 'routes' under 'networking' and the click 'create route'. Copy-paste [this configuration](https://git.dcs.gla.ac.uk/RichardMccreadie/openshift-applications/-/blob/master/tutorial/pythonJupyterNotebookRoute.yaml) into the dialogue window. Be sure to, once again, change 'richardstestproject' to the name of your own openshift cluster project. Once you are finished, click 'create'.

4. Now click on 'deployment configs' under 'workloads' and then click 'create deployment config'. Copy-paste the contents of the `epsrc-2023/epsrc-cluster-files/deploymentconfigs/notebook.yml` into the dialogue. Be sure to replace '2663460avol1claim' with the name of your own persistent storage claim, '2663460a' with the name of your own group and '2663460aproject' with the name of your own openshift cluster project.

5. Give the job's pods 5-10 minutes to start up. Once they are ready, you will find the link to access them with the 'route' you created earlier.

## Simulation Data Preprocessing

1. In order to use the simulation data for neural network training, it must first be preprocessed into a usable set of files. First, navigate to your jupyter notebook instance via an internet browser.

2. Click on 'model.ipynb' on the jupyter notebook interface.

3. Run the first notebook cell, this should execute without any failures.

4. In the second notebook cell, replace '/nfs/sim_data' with the path to the directory containing only the simulation databases to be preprocessed. If the simulation databases are still in the default directory, you should move them now using the instructions above.

5. Run the second cell of the notebook. This will take several hours to execute and should finish without any failure. *Do not run this cell again after this point unless you intend to preprocess the data again.*

6. You should now notice 3 new files under the `epsrc-2023/epsrc-vehicle-model` directory. The two '.bin' files are preprocessed test/train data, whereas the 'constraints.p' file contains serialized data about the distributions of the training data.

## Neural Network Training

1. Navigate to the project's jupyter notebook hosted on the cluster using an internet browser.

2. Click 'model.ipynb' on the jupyter interface.

3. Run the first cell, this should take around 10 seconds to complete without any failures.

4. Skip the second cell and run the **third** cell. This cell should take around 2 minutes to complete without any failures.

5. Run the 4th cell, this cell should complete instantly.

6. You may run the 5th cell to conduct a parameter experiment. The parameters which will be varied can be configured towards the bottom of the cell. Familiarise yourself with the [ray tune]() library to understand how to view the experimental results.

7. Configure the 6th cell so that the model's hyperparameters are to your liking

    | Parameter | Description |
    | - | - |
    | epochs | How many epochs to train for. |
    | num_layers | Number of hidden layers in the model. Greatly increases training time. |
    | num_neurons | Number of neurons in each hidden layer. |
    | show_plots | Whether to show plots in the notebook. Plots will also be saved to a new directory by default. Recommended value is false as there are many plots to show. |

8. Run the cell, a progress bar should indicate how much time is remaining.

9. Click the 'Jupyter' logo in the top-left. You should now see a new directory named with a timestamp. This directory will contain the trained model as well as a variety of plots showing predictions, input distributions, output distributions and training/validation losses.