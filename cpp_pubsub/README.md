ROS2 - Beginner Tutorials - Publisher & Subscriber

This repository contains the publisher-subscriber tutorial from ROS2 Beginner: Client libraries.

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
ros2 run cpp_pubsub talker
```
Running subscriber
```
ros2 run cpp_pubsub listener 
```
