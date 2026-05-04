---
name: clingwrap
description: Use when authoring or editing ROS 2 Python launch files in this workspace - clingwrap is the local Pythonic wrapper around launch/launch_ros that replaces hand-rolled `LaunchDescription([...])` lists with a `LaunchBuilder` (`cw.LaunchBuilder()`) plus context managers for namespaces and composable node containers. Triggers on `*.launch.py`, `generate_launch_description`, `import clingwrap`, `LaunchBuilder`, `composable_node_container`, or any time launch arguments / nodes / namespaces / composable nodes need to be declared.
---

# clingwrap

In-repo Pythonic launch wrapper. Source: `src/deps/clingwrap/clingwrap/`. Reference: `src/deps/clingwrap/README.md`. Test fixtures (good minimal examples): `src/deps/clingwrap/tests/fixtures/`.

Always `import clingwrap as cw`. Build a launch file by instantiating `l = cw.LaunchBuilder()`, calling methods on `l`, and `return l` from `generate_launch_description()`. `LaunchBuilder` *is* a `LaunchDescription`, so just return it.

## Skeleton

```python
from launch import substitutions as sub
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()
    # ... declare args, add nodes, etc ...
    return l
```

`use_sim_time` is auto-declared as a bool launch arg and auto-injected into every node's `parameters` and into `launch_arguments` of every `include_launch_py`. Read it via `l.use_sim_time` (a `LaunchConfiguration`). Don't declare it yourself.

## Launch arguments

- `l.declare_arg(name, default_value=..., description=..., choices=...)` → returns a `LaunchConfiguration` (a substitution). Use that handle directly in node names, params, remappings.
- `l.declare_bool_arg(name, default_value=False)` → same, but constrains choices to `true|false|True|False`. `default_value` is a Python `bool`.

## Nodes

```python
l.node(
    package,
    executable=None,            # defaults to package
    parameters=None,            # dict[str, value-or-substitution]
    parameters_file=None,       # path or substitution (use cw.pkg_file)
    remappings=None,            # dict[from, to]
    log_level=None,             # cw.LogLevel.DEBUG | INFO | WARN | ERROR | FATAL | NONE
    name=..., namespace=..., **node_kwargs,
)
```

- If both `parameters` and `parameters_file` are given, the file is applied first then the dict overrides — order matters in launch_ros.
- `emulate_tty=True` and `output="screen"` are auto-set unless you pass either explicitly. Don't set them by hand.
- `log_level` is sugar for appending `--log-level <level>` to `ros_arguments`.

## Namespaces

```python
with l.namespace("robot1"):
    l.node("driver")
    with l.namespace("sensors"):
        l.node("camera")               # robot1/sensors
    with l.namespace("/robot2"):       # absolute — replaces stack
        l.node("planner")              # /robot2
```

A namespace starting with `/` is absolute and discards everything above it. Namespaces compose via a `GroupAction`+`PushRosNamespace`. Nesting is fine.

## Composable nodes

```python
with l.composable_node_container(
    name="my_container",
    container_type=cw.ContainerType.SINGLE_THREAD_EACH,   # default
    parameters=..., parameters_file=..., remappings=...,
    log_level=..., namespace=..., **container_kwargs,
):
    l.composable_node(
        package, plugin,                # plugin e.g. "pkg::ClassName"
        name=..., parameters=..., parameters_file=..., remappings=...,
        **node_kwargs,
    )
```

Container types (`cw.ContainerType`):
- `SINGLE_THREAD` — one shared single-threaded executor (`component_container`).
- `SINGLE_THREAD_EACH` — each component its own thread (`component_container_isolated`). **Default.**
- `MULTI_THREAD` — one shared multi-threaded executor (`component_container_mt`).
- `MULTI_THREAD_EACH` — isolated, multi-threaded per component.

Only one container can be open at a time — nesting `composable_node_container` raises `ValueError`. Calling `l.composable_node(...)` outside a container also raises.

## Topic tools

Both auto-spawn their own composable container if you call them outside one, otherwise they slot into the open container. Both use `topic_tools` (declared in `clingwrap`'s `package.xml` as exec_depend, no per-package addition needed).

```python
l.topic_relay(from_topic, to_topic, lazy=True)
l.topic_throttle_hz(topic, rate, lazy=True, include_hz_in_output_topic=False)
# throttle output: "<topic>/throttled"  (Hz suffix is templated; see note below)
```

Note: `include_hz_in_output_topic=True` currently emits the literal string `"/hz_{friendly_rate}"` (unformatted) — see `launch_builder.py:306`. Avoid it until fixed, or pass the desired suffix yourself.

## Including other launch files

```python
l.include_launch_py(
    package="some_pkg",
    launch_file="thing.launch.py",
    launch_arguments={"x": "1"},      # auto-augmented with use_sim_time
    directory="launch",                # default
)
```

Only `.launch.py` (PythonLaunchDescriptionSource) is supported via this helper. For non-Python sources, fall back to raw `launch.actions.IncludeLaunchDescription` and feed it in via `l._action_list.add_action(...)` (or rethink).

`l.include_actions_from_launch_description(ld)` copies the `Action` entities out of an existing `LaunchDescription` into the current builder.

## Logging & opaque functions

```python
l.log("static message")
l.log(["dynamic: ", some_substitution])
l.opaque_function(lambda ctx: [<list of Actions>])   # for runtime-only logic
```

## Helpers (`launch_helpers.py`, `param_helpers.py`)

- `cw.pkg_file(package, *path_parts)` → `PathJoinSubstitution([FindPackageShare(package), ...])`. Use for any `parameters_file=` or share-relative path.
- `cw.as_str_param(value)` → wraps with `ParameterValue(value, value_type=str)`. Use when a node expects a string but the value would otherwise be coerced numerically/booleanly (URDF strings, version strings, numeric IDs that must remain strings).
- `cw.remap_action(from_, to)` → returns the dict needed to fully remap a ROS 2 action (workaround for ros2/ros2#1312). Spread it: `remappings={**cw.remap_action("a", "b"), ...}`.
- `cw.remap_hidden(topic)` → `{topic: f"_hidden_{topic.strip('/')}"}` — hides a topic from `ros2 topic list` introspection.
- `cw.cov_xyz_diag(x, y, z)` → 9-element row-major diagonal covariance.

## Static analysis

`l.get_static_information()` returns a `StaticInformation` (`static_info.py`) with:
- `nodes: list[NodeInfo]`
- `composable_node_containers: list[ComposableNodeContainerInfo]` — each has `nodes: list[ComposableNodeInfo]`
- `included_launch_files: list[LaunchFileInclude]`
- `.get_all_composable_nodes()` flattens containers
- `.merge(other)` combines two `StaticInformation`s

Each tracked entity carries the resolved namespace from the namespace stack at call time (relative joined with `/`, absolute resets the stack). Substitutions are stored unresolved — don't expect raw strings for substitution-valued fields.

## Common gotchas

- `use_sim_time` is added to *every* node's parameters. If a node's params are a non-dict (e.g. a list pulled from elsewhere), thread it through a dict instead — `_add_sim_time` mutates the passed dict in place and creates `{}` if `None`.
- `parameters_file` argument name is singular and takes one entry. To layer multiple files, drop into raw `launch_ros` for that node, or merge externally.
- `name`/`namespace` go via `**node_kwargs`, not as named keyword args of `l.node` — they're forwarded straight to `launch_ros.actions.Node`.
- Default container type is `SINGLE_THREAD_EACH` (isolated), **not** `SINGLE_THREAD`. Pick deliberately.
- Don't subclass or instantiate `LaunchDescription` directly in this repo — always `cw.LaunchBuilder()`.
