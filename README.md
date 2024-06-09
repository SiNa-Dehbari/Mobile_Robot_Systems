# Mobile_Robot_Systems

A ROS2 package to receive order_ids, arrange them and finding the shortest path to collect parts and deliver to the destination.

## Author:
[Sina Dehbari](dehbari.sina@gmail.com)


## Reference:

[Changelog](CHANGELOG.md)




## Requirements:

Ubuntu 22.04 LTS

ROS2 Humble (at least Desktop version - see [ROS2 wiki](https://docs.ros.org/en/humble/index.html))


## Setup Dependencies:
clone this repository and install the required dependencies.

TODO

## How does it work?

After installing the dependencies and cloning this package in to your workspace, run the code as below:

```
ros2 run mobile_robot_systems OrderOptimizer --ros-args -p directory_path:=/path/to/the/directory
```
Note: if the directory is not provided for the node, it gets the default directory (src/mobile_robot_systems/files)

## Parameters

This pacakge includes three nodes:

|  node   |  type | value  |  summary |
| --------- | ------- | -------  | ------------- |
| pose_subscriber_ | subscriber | PoseStamped | position of the Robot |
| oder_subscriber_ | subscriber | NextOrder | subscribe to the next order |
| marker_array_publisher_ | publisher | MarkerArray | publishing location of robot as well as part |

NextOrder is a customized message inside this file with below infos:

uint32 order_id
string description

## Test and Deploy

TODO
Use the built-in continuous integration in GitLab.


## Project status
This is the Sample work for company [Knapp](https://www.knapp.com/en/) to evaluate coding style.