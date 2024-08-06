# Build and Launch Pointcloud_Perception and Launch Docker container

## 
### Build
To build Pointcloud_Perception:
```
$ colcon build --parallel-workers 1`
```
It's possible to just run `colcon build` but may make the system run out of memory.

### Launch
To launch the package:
```
$ source install/setup.bash
$ ros2 launch launch/pointcloud_perception.xml
```
## Docker container Launch

### Requirement

Have a NVIDIA graphic card and nvidia-container-toolkit [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed

### Build and Launch
```
$ docker build -t <image_name> .
$ docker run -it --gpus all <image_name>
```



Once in the container:

```
$ source install/setup.bash
$ ros2 launch launch/pointcloud_perception.xml
```
