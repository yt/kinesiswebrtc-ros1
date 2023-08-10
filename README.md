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
## Build
You need to install [aws-sdk-cpp](https://github.com/aws/aws-sdk-cpp). Since ros environments tend to rebuild too frequently I have removed the old sdk setup from aws_common package.
<br/>While installing the sdk; `cmake <path-to-root-of-aws-sdk> -DCMAKE_BUILD_TYPE=Release -DBUILD_ONLY="core;kinesis"` is enaugh.

