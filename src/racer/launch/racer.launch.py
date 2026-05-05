import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        package="autodrive_roboracer",
        executable="autodrive_bridge",
    )

    with l.namespace("perception"):
        l.node(
            package="perception",
            executable="wall_extractor",
            remappings={"scan": "/autodrive/roboracer_1/lidar"},
        )

    return l
