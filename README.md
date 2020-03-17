# Kimera-VIO-ROS2

ROS2 Wrapper for [Kimera](https://github.com/MIT-SPARK/Kimera).


## Docker Demo

To run this demo using docker, the following dependencies are required:

* [ubuntu](https://ubuntu.com/)
  * Login in using x11 for simple display forwarding with containers.
  * Other linux distributions may work, but we'll focus on ubuntu.
* [docker](https://www.docker.com/)
  * Quick install script here:
  * https://get.docker.com
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)
  * For hardware accelerated graphics in container
* [rocker](https://github.com/osrf/rocker)
  * Check display forwarding is working with rocker.
* [off-your-rocker](https://github.com/sloretz/off-your-rocker)
  * Rocker extension used to pass arbitrary docker arguments.

> Note: for manual host installation, follow the general build steps in the [Dockerfile](Dockerfile).


### Building Demo:

Clone then navigate into the repo folder and run the following command to get the Docker image

```
docker pull ruffsl/kimera_vio_ros2
# or
docker build --tag ruffsl/kimera_vio_ros2 .

```

This may take some time, so go grab a cup of coffee.


### Running Demo

Once you have the docker image, use rocker to launch a container with GUI display support

```
rocker --x11 --nvidia \
  --oyr-run-arg " --net=host --ipc=host --pid=host" \
  ruffsl/kimera_vio_ros2 \
  ros2 launch kimera_vio_ros realsense_ir.launch.xml
```

In this case, you'll also need to launch a ros2 realsense node to acquire sensor data.

> Note: the linked fork below also includes a Dockerfile for reproducible builds.

* [librealsense](https://github.com/IntelRealSense/librealsense)
  * SDK library for Intel RealSense sensors
* [ros2_intel_realsense](https://github.com/ruffsl/ros2_intel_realsense/)
  * Current fork with patches for better VIO.

Given the loading of kernel modules needed for accurate frame timestamps can not be achieved using docker alone, please be sure to at least install `librealsense2-dkms` or build and load the dkms from source onto your host kernel. Otherwise you'll likely encounter degraded timestamp as mentioned [here](https://github.com/IntelRealSense/librealsense/issues/5710)

```
docker pull ruffsl/ros2_intel_realsense:master
# or
git clone https://github.com/ruffsl/ros2_intel_realsense.git
cd ros2_intel_realsense
docker build --tag ruffsl/ros2_intel_realsense:master .
```

Once you have the docker image, use `--privileged` to launch a container with USB device access:

```
docker run -it --rm --privileged \
  --net=host --ipc=host --pid=host \
  ruffsl/ros2_intel_realsense:master \
  ros2 run realsense_node realsense_node \
    --ros-args -p depth0.emitter:=0
```

> Note: `--net=host --ipc=host --pid=host` is used to allow ROS2/DDS to communicate across containers.
> More details on connectivity across containers can be found [here](https://answers.ros.org/question/296828/ros2-connectivity-across-docker-containers-via-host-driver/).