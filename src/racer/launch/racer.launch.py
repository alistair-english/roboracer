import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        package="autodrive_roboracer",
        executable="autodrive_bridge",
    )

    return l
