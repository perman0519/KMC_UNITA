import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # 패키지 경로 및 설정 파일 경로 정의
    pkg_share = get_package_share_directory('pure_pursuit_controller')
    
    # 설치 시 config 폴더가 share/pure_pursuit_controller/config에 들어가므로 해당 경로 사용
    bridge_1_to_2_cfg = os.path.join(pkg_share, 'config', '1_to_2_bridge.yaml')
    bridge_2_to_1_cfg = os.path.join(pkg_share, 'config', '2_to_1_bridge.yaml')
    hv_quiz2_bridge = os.path.join(pkg_share, 'config', 'hv_quiz2_bridge.yaml')
    quiz3bridge = os.path.join(pkg_share, 'config', 'quiz3bridge.yaml')
    bridge3 = os.path.join(pkg_share, 'config', '3bridge.yaml')
    bridge4 = os.path.join(pkg_share, 'config', '4bridge.yaml')

    problem_id = LaunchConfiguration('problem_id')

    return LaunchDescription([
        DeclareLaunchArgument('problem_id', default_value='1-1'),

        # PROBLEM_ID=3 -> pure_pursuit_node 1-1
        Node(
            condition=LaunchConfigurationEquals('problem_id', '1-1'),
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '1'},
        ),

        # [PROBLEM_ID=4 일 때만 실행되는 그룹]
        
        # 1. Domain Bridge (1 -> 2)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '1-2'),
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_1_to_2',
            arguments=[bridge_1_to_2_cfg],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),

        # 2. Domain Bridge (2 -> 1)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '1-2'),
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_2_to_1',
            arguments=[bridge_2_to_1_cfg],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),

        # 3. Quiz 1-2 CAV1 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '1-2'),
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '1'},
        ),
        # 3. Quiz 1-2 CAV2 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '1-2'),
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '2'},
        ),
        
        # PROBLEM_ID=2 (문제 2)
        # Domain Bridge (100 -> 1)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '2'),
            package='domain_bridge',
            executable='domain_bridge',
            name='hv_quiz2_bridge',
            arguments=[hv_quiz2_bridge],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),
        # Quiz 2 CAV1 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '2'),
            package='pure_pursuit_controller',
            executable='quiz2_node',
            name='quiz2_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '1'},
        ),
        
        # PROBLEM_ID=3 (문제 3) 
        # Domain Bridge (100 -> 1,2,3,4)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='domain_bridge',
            executable='domain_bridge',
            name='quiz3bridge',
            arguments=[quiz3bridge],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),
        # Domain Bridge (1 -> 2,3,4)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_1_to_2',
            arguments=[bridge_1_to_2_cfg],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),
        # Domain Bridge (2 -> 1,3,4)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_2_to_1',
            arguments=[bridge_2_to_1_cfg],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),
        # Domain Bridge (3 -> 1,2,4)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='domain_bridge',
            executable='domain_bridge',
            name='bridge3',
            arguments=[bridge3],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),
        # Domain Bridge (4 -> 1,2,3)
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='domain_bridge',
            executable='domain_bridge',
            name='bridge4',
            arguments=[bridge4],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '100'},
        ),
        # Quiz 3 CAV1 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='pure_pursuit_controller',
            executable='quiz3_node',
            name='quiz3_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '1'},
        ),
        # Quiz 3 CAV2 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='pure_pursuit_controller',
            executable='quiz3_node',
            name='quiz3_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '2'},
        ),
        # Quiz 3 CAV3 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='pure_pursuit_controller',
            executable='quiz3_node',
            name='quiz3_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '3'},
        ),
        # Quiz 3 CAV4 Node
        Node(
            condition=LaunchConfigurationEquals('problem_id', '3'),
            package='pure_pursuit_controller',
            executable='quiz3_node',
            name='quiz3_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '4'},
        ),
    ])