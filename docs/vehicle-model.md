# vehicle-model

This submodule contains a library with pytorch Neural Network for learning vehicle dynamics as well as a jupyter notebook with code for training the network and running parameter experiments with ray.

For normal usage, follow the [cluster-files](./cluster-files.md) documentation to deploy the jupyter notebook on the cluster, or alternatively, try running it locally.

Raw simulation data must first be preprocessed. This can take upwards of a couple of hours on larger datasets. In the preprocessing stage, data is split into x-y tensor pairs and serialized in a binary format.

Every time the notebook is restarted, the datasets must be loaded back into memory using the second notebook cell.

The train test loop automatically saves all plots, the model .pt file and parameters to a timestamped folde rin the notebook's directory.

Parameter experiments are not automatically saved.