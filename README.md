# Dataset Builder
ROS package to conintuously capture multiple sensor streams from the HoloLens 2, for building datasets for the HRI-Cacti project.

## Run
1. Launch ROS Nodes
   ```shell
   roslaunch dataset_builder build.launch classname:="Advance"
   ```
## Compressing
   ```shell
   # compress
   tar czpvf - /path/to/archive | split -d -b 100M - tardisk
   ```
   ```shell
   # uncompress
   cat tardisk* | tar xzpvf -
   ```
