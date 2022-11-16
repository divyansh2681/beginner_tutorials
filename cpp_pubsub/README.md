ROS2 - Beginner Tutorials - Publisher & Subscriber

This repository contains the publisher-subscriber tutorial from ROS2 Beginner: Client libraries.

Dependencies:
-Ubuntu 20.04
-ROS2 Foxy Fitzroy

Compiling and Running:

(`code`)
cd "your workspace"/src
git clone https://github.com/divyansh2681/beginner_tutorials.git
cd ..
colcon build --packages-select cpp_pubsub
source "your workspace"/install/setup.bash
(`code`)

Running publisher
 
(`code`) ros2 run cpp_pubsub talker (`code`)

Running subscriber

(`code`) ros2 run cpp_pubsub listener (`code`)
