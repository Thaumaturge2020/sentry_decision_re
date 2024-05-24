from launch import LaunchDescription
from launch_ros.actions import Node

package_dir = "/home/ubuntu/sentry_ws/src/sentry_decision/src/topic_pub_node"

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="topic_pub_node",
                executable="topic_pub_node_main_2",
                name="topic_pub_node_2",
                output="screen",
                # parameters=[{"agoalX": 14.0}, {"agoalY": 14.0}, {"useTest": True}],
                parameters=[
                ],
            ),


            Node(
                package="topic_pub_node",
                executable="topic_pub_node_main_1",
                name="topic_pub_node_1",
                output="screen",
                # parameters=[{"agoalX": 14.0}, {"agoalY": 14.0}, {"useTest": True}],
                parameters=[
                ],
            ),
        ]
    )
