from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    # Map Visualizer Node
    nodes.append(
        Node(
            package='map_visualizer',
            executable='map_visualizer_node',
            name='map_visualizer_node',
            output='screen'
        )
    )

    # CAV Visualizer Nodes (1 to 4)
    for i in range(1, 5):
        topic_name = f'/CAV_{i:02d}' # /CAV_01, /CAV_02...
        nodes.append(
            Node(
                package='map_visualizer',
                executable='cav_visualizer_node',
                name=f'cav_visualizer_node_{i}',
                parameters=[
                    {'topic_name': topic_name},
                    {'vehicle_id': i}
                ],
                output='screen'
            )
        )

    # RViz2
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ''], # User can save config later
            output='screen'
        )
    )

    return LaunchDescription(nodes)