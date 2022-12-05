ROS2 - Beginner Tutorials - Publisher, Subscriber, Services, tf2, unit testing & bag files

This repository contains the publisher-subscriber tutorial from ROS2 Beginner: Client libraries and Services also.

Dependencies:
<ul>
<li> Ubuntu 20.04 </li>
<li> ROS2 Foxy Fitzroy </li>
</ul>

Compiling and Running:
```
cd "your workspace"/src
git clone https://github.com/divyansh2681/beginner_tutorials.git
cd ..
colcon build --packages-select cpp_pubsub
source "your workspace"/install/setup.bash
```

Running publisher
```
ros2 run cpp_pubsub talker queue:=40
```
Running subscriber
```
ros2 run cpp_pubsub listener 
```
Calling the client
```
  ros2 run cpp_pubsub clienttt FATAL
```
Using the launch file with an argument
```
  ros2 launch cpp_pubsub launchFile.yaml queue:=40
```
Running tf2 broadcaster
```
ros2 run tf2_ros tf2_echo world talk
```
Recording bag file for ROS
```
ros2 launch cpp_pubsub launch_week11.py record:=True

ros2 bag play <bag_file_name>
ros2 run cpp_pubsub listener
```


Note: When running the client, FATAL is just an example. You can use any logger level (INFO, ERROR, DEBUG, WARN).