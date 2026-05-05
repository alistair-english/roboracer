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
        # l.node(
        #     package="perception",
        #     executable="midline_tracer",
        #     parameters={
        #         "track_width": 2.5,
        #         "cast_cap": 1.5,
        #     }
        # )

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
            }
        )

    return l
