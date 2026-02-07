from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # 1. Launch Arguments 선언
    ld = LaunchDescription([
        DeclareLaunchArgument('cav1_id', default_value='1'),
        DeclareLaunchArgument('cav2_id', default_value='2'),
        DeclareLaunchArgument('cav3_id', default_value='3'),
        DeclareLaunchArgument('cav4_id', default_value='4'),
    ])

    # Map Visualizer Node
    ld.add_action(
        Node(
            package='map_visualizer',
            executable='map_visualizer_node',
            name='map_visualizer_node',
            output='screen'
        )
    )

    # 2. CAV Visualizer Nodes 설정
    cav_configs = [
        {'logical_num': 1, 'id_sub': LaunchConfiguration('cav1_id')},
        {'logical_num': 2, 'id_sub': LaunchConfiguration('cav2_id')},
        {'logical_num': 3, 'id_sub': LaunchConfiguration('cav3_id')},
        {'logical_num': 4, 'id_sub': LaunchConfiguration('cav4_id')},
    ]

    for config in cav_configs:
        # [수정 포인트] PythonExpression 내에서 zfill(2)를 사용하여 '05' 처럼 만듦
        # 주의: 홑따옴표와 쌍따옴표 구분을 잘 해야 합니다.
        topic_name_expr = PythonExpression([
            "'/CAV_' + str(", config['id_sub'], ").zfill(2)"
        ])

        display_name_expr = PythonExpression([
            "'CAV_' + str(", config['id_sub'], ")"
        ])

        ld.add_action(
            Node(
                package='map_visualizer',
                executable='cav_visualizer_node',
                name=f"cav_viz_slot_{config['logical_num']}",
                parameters=[
                    {'topic_name': topic_name_expr},       # /CAV05_pose 등
                    {'logical_id': config['logical_num']}, # 1, 2, 3, 4 고정
                    {'display_name': display_name_expr}    # 마커 위 이름 (CAV_5)
                ],
                output='screen'
            )
        )

    ld.add_action(
        Node(
            package='map_visualizer',
            executable='cav_visualizer_node',
            name=f"cav_viz_slot_{config['logical_num']}",
            parameters=[
                {'topic_name': '/HV_19'},       # /CAV05_pose 등
                {'logical_id': 19}, # 1, 2, 3, 4 고정
                {'display_name': 'HV_19'}    # 마커 위 이름 (CAV_5)
            ],
            output='screen'
        )
    )

    ld.add_action(
        Node(
            package='map_visualizer',
            executable='cav_visualizer_node',
            name=f"cav_viz_slot_{config['logical_num']}",
            parameters=[
                {'topic_name': '/HV_20'},       # /CAV05_pose 등
                {'logical_id': 20}, # 1, 2, 3, 4 고정
                {'display_name': 'HV_20'}    # 마커 위 이름 (CAV_5)
            ],
            output='screen'
        )
    )
    return ld
