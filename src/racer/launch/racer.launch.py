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
        l.node(
            package="perception",
            executable="midline_tracer",
            parameters={
                "track_width": 1.8,
            }
        )

    return l
