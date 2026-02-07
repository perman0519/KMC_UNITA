from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    communication_manager = ExecuteProcess(
        cmd=['ros2', 'run', 'communication_manager', 'pose_sharing_handler'],
        output='screen'
    )

    hv_path_follower = ExecuteProcess(
        cmd=['ros2', 'run', 'hv_handler', 'hv_handler_node'],
        output='screen'
    )

    simulator = ExecuteProcess(
        cmd=['ros2', 'run', 'simulator', 'simulation_node'],
        output='screen'
    )

    order_two = RegisterEventHandler(
        OnProcessStart(
            target_action=communication_manager,
            on_start=[hv_path_follower]
        )
    )

    order_three = RegisterEventHandler(
        OnProcessStart(
            target_action=hv_path_follower,
            on_start=[simulator]
        )
    )

    return LaunchDescription([
        communication_manager,
        order_two,
        order_three,
    ])
