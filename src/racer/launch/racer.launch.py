from pathlib import Path

import clingwrap as cw
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    l = cw.LaunchBuilder()

    racer_urdf = Path(get_package_share_directory("racer")) / "urdf" / "racer.urdf"

    launch_rviz = l.declare_bool_arg("rviz", default_value=False)

    # autodrive_bridge publishes ground-truth TF, which the rules forbid us from
    # using at race time. Shunt it onto /sim/tf so the default /tf is reserved
    # for our own SLAM/odom tree and nothing else can accidentally consume it.
    l.node(
        package="autodrive_roboracer",
        executable="autodrive_bridge",
        remappings={"/tf": "/sim/tf", "/tf_static": "/sim/tf_static"},
    )

    l.node(
        package="robot_state_publisher",
        parameters={"robot_description": racer_urdf.read_text()},
    )

    l.node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", cw.pkg_file("racer", "config", "racer_view.rviz")],
        condition=IfCondition(launch_rviz),
    )

    with l.namespace("perception"):
        l.node(
            package="perception",
            executable="wall_extractor",
            remappings={"scan": "/autodrive/roboracer_1/lidar"},
        )

    with l.namespace("control"):
        l.node(
            package="control",
            executable="tentacles",
            remappings={
                "walls": "/perception/walls",
                "steering_command": "/autodrive/roboracer_1/steering_command",
                "throttle_command": "/autodrive/roboracer_1/throttle_command",
            },
            parameters={
                "min_throttle": 0.01,
                "max_throttle": 0.02,
            },
        )

    return l
