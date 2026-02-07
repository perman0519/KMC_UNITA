import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 공유 디렉토리 경로 가져오기 (설치된 경로 기준)
    # 만약 소스 경로를 직접 참조하고 싶다면 아래 경로를 직접 수정하세요.
    pkg_dir = get_package_share_directory('pure_pursuit_controller')

    # 1. 1_to_2_bridge
    bridge_11 = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge_11',
        arguments=[os.path.join(pkg_dir, 'config', 'cav1_bridge.yaml')]
    )

    # 2. 2_to_1_bridge
    bridge_22 = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge_22',
        arguments=[os.path.join(pkg_dir, 'config', 'cav2_bridge.yaml')]
    )

    # 3. 3bridge
    bridge_33 = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge_33',
        arguments=[os.path.join(pkg_dir, 'config', 'cav3_bridge.yaml')]
    )

    # 4. 4bridge
    bridge_44 = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge_44',
        arguments=[os.path.join(pkg_dir, 'config', 'cav4_bridge.yaml')]
    )

    # # 5. hv_19_20_to_cavs
    # bridge_1920 = Node(
    #     package='domain_bridge',
    #     executable='domain_bridge',
    #     name='domain_bridge_1920',
    #     arguments=[os.path.join(pkg_dir, 'config', 'quiz3bridge.yaml')]
    # )

    return LaunchDescription([
        bridge_11,
        bridge_22,
        bridge_33,
        bridge_44,
    
    ])
