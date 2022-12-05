from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import sys

def generate_launch_description():
    rosbag_record = sys.argv[-1]
    actions_list = [
        Node(
            package='cpp_pubsub',
            executable='talker',
            arguments = ['0', '0', '1', '0', '0', '0', 'world', 'talk']
        ),
        Node(
            package='cpp_pubsub',
            executable='listener'
        )
    ]
    if rosbag_record=='record:=True':
        actions_list.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", "cpp_pubsub_bag", "-a"]
            )
        )
    return LaunchDescription(actions_list)