# Amazon Kinesis Video Streams WebRTC on ROS 1

## Overview
The Kinesis Webrtc ROS package enables ROS image topics to transmit video using AWS Kinesis Video Streams with WebRTC.
This repository is the combination of below repositories and fitted to use with ROS `noetic & melodic`
```
https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c
https://github.com/aws-robotics/kinesisvideo-ros1
https://github.com/aws-robotics/utils-ros1
https://github.com/aws-robotics/kinesisvideo-encoder-common
https://github.com/aws-robotics/utils-common
```
### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Clone the repo 
- Install dependencies

        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y

- Build the packages

        catkin build

- Source ROS library Path

### Run the application
```
roslaunch h264_video_encoder sample_application.launch
rosrun kinesis_video_webrtc kvsWebrtcClientMaster
```
_Note: I have only converted kvsWebrtcClientMaster.c to ROS code for now._


### Build and run the unit tests
```
catkin build --cmake-target tests
catkin test
```

## Launch Files
A launch file called `h264_video_encoder.launch` is included in this package. The launch file uses the following arguments:

| Arg Name | Description |
| --------- | ------------ |
| node_name | (optional) The name the H264 encoder node should be launched with. If not provided, the node name will default to `h264_video_encoder` |
| config_file | (optional) A path to a rosparam config file. |

An example launch file called `sample_application.launch` is included in this project that gives an example of how you can include this node in your project and provide it with arguments.


## Usage

### Running the node
To launch the H264 encoder node, you can run the following command:

    roslaunch h264_video_encoder sample_application.launch


## Configuration File and Parameters
An example configuration file called `sample_configuration.yaml` is provided for running the H264 encoder node on a Raspberry Pi based system.
When the parameters are absent in the ROS parameter server, default values are used, thus all parameters are optional. See table below for details.

| Parameter Name | Description | Type |
| ------------- | -----------------------------------------------------------| ------------- |
| queue_size | (optional) The maximum number of incoming and outgoing messages to be queued towards the subscribed and publishing topics. | integer |
| output_width | (optional) The desired width (in pixels) of each frame in the encoded video output. | integer |
| output_height | (optional) The desired height (in pixels) of each frame in the encoded video output. | integer |
| fps_numerator | (optional) The desired frames per second (the numerator portion when expressing FPS as a rational number) for the encoded video output. | integer |
| fps_denominator | (optional) The desired frames per second (the denominator portion when expressing FPS as a rational number) for the encoded video output. | integer |
| bitrate | (optional) The desired bitrate (in bits per second) of the encoded video output. | integer |


## Node Details

#### Published Topics
| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| *Configurable* (default="video/encoded") | kinesis_video_msgs/KinesisVideoFrame | The node will publish to a topic of a given name. Each message being published contains a chunk of the video stream, usually per video frame. |

#### Subscribed Topics
| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| *Configurable* (default="/raspicam_node/image") | sensor_msgs/Image | The node will subscribe to a topic of a given name. The data is expected to be a stream of images from a source (such as a Raspberry Pi camera). |

