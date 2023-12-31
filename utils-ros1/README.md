# AWS Utils Library for ROS1


## Overview
This is the common library for all of AWS RoboMaker ROS1 packages.
It builds on top of the generic interfaces defined by [utils-common](https://github.com/aws-robotics/utils-common) to provide ROS1-specific helper functions and classes implementations for things such as logging and parameter loading (see [usage example](https://github.com/aws-robotics/cloudwatchmetrics-ros1/blob/master/cloudwatch_metrics_collector/src/cloudwatch_metrics_collector.cpp#L138,L139)).

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Kinetic
- Melodic

### Build status
* Travis CI:
    * "master" branch [![Build Status](https://travis-ci.org/aws-robotics/utils-ros1.svg?branch=master)](https://travis-ci.org/aws-robotics/utils-ros1/branches)
    * "release-latest" branch [![Build Status](https://travis-ci.org/aws-robotics/utils-ros1.svg?branch=release-latest)](https://travis-ci.org/aws-robotics/utils-ros1/branches)
* ROS build farm:
    * ROS Kinetic @ u16.04 Xenial [![Build Status](http://build.ros.org/job/Kbin_uX64__aws_ros1_common__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__aws_ros1_common__ubuntu_xenial_amd64__binary)
    * ROS Melodic @ u18.04 Bionic [![Build Status](http://build.ros.org/job/Mbin_uB64__aws_ros1_common__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__aws_ros1_common__ubuntu_bionic_amd64__binary)


## Installation

### Binaries
On Ubuntu you can install the latest released version of this package using the following command

        sudo apt-get update
        sudo apt-get install -y ros-$ROS_DISTRO-aws-ros1-common

### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Create a ROS workspace and a source directory: `mkdir -p ~/ros-workspace/src`

- Clone the package into the source directory . 

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/utils-ros1.git -b release-latest

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library path

        source ~/ros-workspace/install/setup.bash


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
