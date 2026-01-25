from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 설정 정보
    package_name = 'pure_pursuit_controller'  # 실제 패키지 이름으로 확인해 주세요
    executable_name = 'quiz3_node'
    domain_ids = ['5', '6', '7', '8']

    nodes = []

    for d_id in domain_ids:
        # 각 도메인 ID별로 노드 설정 생성
        nodes.append(
            Node(
                package=package_name,
                executable=executable_name,
                # 노드 이름이 중복되지 않도록 도메인 ID를 붙여줌
                name=f'quiz3_node_domain_{d_id}',
                # 핵심: 해당 노드 프로세스에만 특정 ROS_DOMAIN_ID 부여
                additional_env={'ROS_DOMAIN_ID': d_id},
                output='screen'
            )
        )

    return LaunchDescription(nodes)
