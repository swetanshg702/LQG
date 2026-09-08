import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("quadrotor_gnc")

    estimator_params = os.path.join(
        pkg_share, "config", "estimator_params.yaml")

    controller_params = os.path.join(
        pkg_share, "config", "controller_params.yaml")

    plant_params = os.path.join(
        pkg_share, "config", "plant_params.yaml")

    plant_node = Node(
        package="quadrotor_gnc",
        executable="plant_node",
        parameters=[plant_params]
    )

    estimator_node = Node(
        package="quadrotor_gnc",
        executable="estimator_node",
        parameters=[estimator_params]
    )

    controller_node = Node(
        package="quadrotor_gnc",
        executable="controller_node",
        parameters=[controller_params]
    )

    return LaunchDescription([
        plant_node,
        estimator_node,
        controller_node
    ])