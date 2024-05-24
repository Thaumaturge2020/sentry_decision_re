from launch import LaunchDescription
from launch_ros.actions import Node

package_dir = "/home/ubuntu/sentry_ws/src/sentry_decision/src/behavior_tree_2"

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="behavior_tree_2",
                executable="btree",
                name="behavior_tree_node",
                output="screen",
                # parameters=[{"agoalX": 14.0}, {"agoalY": 14.0}, {"useTest": True}],
                parameters=[
                ],
            ),
            Node(
                package="behavior_tree_2",
                executable="topic_transmit_node",
                name="topic_transmit_node",
                output="screen",
                # parameters=[{"agoalX": 14.0}, {"agoalY": 14.0}, {"useTest": True}],
                parameters=[
                ],
            ),
        ]
    )
