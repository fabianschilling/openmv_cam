# OpenMV Cam ROS package

This package provides an interface between the [OpenMV Cam](https://openmv.io/) and [ROS](http://www.ros.org/).

Images are acquired by the camera and sent via USB to the ROS node which relays them as ROS image messages.

## Getting started

Connect the camera and copy the [main.py](util/main.py) file, either using the [OpenMV IDE](https://openmv.io/pages/download) or directly using the mounted storage device.
Remember to reset the camera via the IDE or eject the storage device to prevent data loss.

Launch the node as follows:

```sh
roslaunch openmv_cam openmv_cam.launch
```

By default, the node uses the following parameters:

```sh
roslaunch openmv_cam openmv_cam.launch device:=/dev/ttyACM0 topic:=/openmv_cam/image_raw compressed:=false
```

## License

The OpenMV Cam ROS package is released under the [MIT License](LICENSE.md).