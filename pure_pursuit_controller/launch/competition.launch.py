import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_controller')

    # 1. Launch Arguments
    ld = LaunchDescription([
        DeclareLaunchArgument('problem_id', default_value='3'),
        DeclareLaunchArgument('cav1_id', default_value='1'),
        DeclareLaunchArgument('cav2_id', default_value='2'),
        DeclareLaunchArgument('cav3_id', default_value='3'),
        DeclareLaunchArgument('cav4_id', default_value='4'),
        DeclareLaunchArgument('target_cav', default_value='all'), # all, cav1, cav2, cav3, cav4
    ])

    problem_id = LaunchConfiguration('problem_id')
    target_cav = LaunchConfiguration('target_cav')

    cav_configs = [
        {'id': LaunchConfiguration('cav1_id'), 'suffix': 'cav1'},
        {'id': LaunchConfiguration('cav2_id'), 'suffix': 'cav2'},
        {'id': LaunchConfiguration('cav3_id'), 'suffix': 'cav3'},
        {'id': LaunchConfiguration('cav4_id'), 'suffix': 'cav4'},
    ]

    for cav in cav_configs:
        # 실행 조건: Problem 3 AND (target이 'all'이거나 현재 cav_suffix와 일치할 때)
        run_condition = PythonExpression([
            "'", problem_id, "' == '3' and ('", target_cav, "' == 'all' or '", target_cav, "' == '", cav['suffix'], "')"
        ])

        # YAML 경로 조립 (PathJoinSubstitution 사용이 가장 안전합니다)
        # cav['id']가 정수형으로 들어올 경우를 대비해 str() 처리가 포함된 표현식 사용
        bridge_cfg = PathJoinSubstitution([
            pkg_share, 'config',
            PythonExpression(["'cav' + str(", cav['id'], ") + '_bridge.yaml'"])
        ])

        # 1. Domain Bridge
        ld.add_action(Node(
            condition=IfCondition(run_condition),
            package='domain_bridge',
            executable='domain_bridge',
            name=f"domain_bridge_{cav['suffix']}",
            arguments=[bridge_cfg],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ))

        # 2. Quiz 3 Node
        ld.add_action(Node(
            condition=IfCondition(run_condition),
            package='pure_pursuit_controller',
            executable='quiz3_node',
            name=f"quiz3_node_{cav['suffix']}",
            output='screen',
            additional_env={'ROS_DOMAIN_ID': cav['id']},
        ))

    return ld
