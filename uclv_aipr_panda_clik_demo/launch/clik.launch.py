from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    clik_node = Node(
        package="uclv_aipr_panda_clik_demo",
        executable="clik",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    joint_traj_server = Node(
        package="uclv_aipr_panda_clik_demo",
        executable="joint_traj_action_server",
        output="screen"
    )

    cartesian_traj_server = Node(
        package="uclv_aipr_panda_clik_demo",
        executable="cartesian_traj_action_server",
        output="screen"
    )

    return LaunchDescription([
        clik_node,
        joint_traj_server,
        cartesian_traj_server
        ])
