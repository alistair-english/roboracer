from typing import Optional

import clingwrap as cw
from launch import SomeSubstitutionsType
from launch.conditions import IfCondition

TF_REMAPPINGS: dict[SomeSubstitutionsType, SomeSubstitutionsType] = {
    "/tf": "/racer/tf",
    "/tf_static": "/racer/tf_static",
}


class RacerLaunchBuilder(cw.LaunchBuilder):
    """LaunchBuilder that injects /tf and /tf_static remappings by default.

    The sim publishes ground-truth poses on /tf, which competition rules forbid
    us from using at race time. Routing every node we own onto /racer/tf*
    isolates our SLAM/localization tree from the sim's. Caller-supplied
    remappings override these defaults.
    """

    @staticmethod
    def _with_tf(
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]],
    ) -> dict[SomeSubstitutionsType, SomeSubstitutionsType]:
        merged = dict(TF_REMAPPINGS)
        if remappings:
            merged.update(remappings)
        return merged

    def node(
        self,
        *args,
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **kwargs,
    ):
        return super().node(*args, remappings=self._with_tf(remappings), **kwargs)

    def composable_node(self, *args, remappings=None, **kwargs):
        return super().composable_node(
            *args, remappings=self._with_tf(remappings), **kwargs
        )

    def composable_node_container(self, *args, remappings=None, **kwargs):
        return super().composable_node_container(
            *args, remappings=self._with_tf(remappings), **kwargs
        )


def generate_launch_description():
    l = RacerLaunchBuilder()

    launch_rviz = l.declare_bool_arg("rviz", default_value=False)

    l.node(
        package="autodrive_roboracer",
        executable="autodrive_bridge",
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
