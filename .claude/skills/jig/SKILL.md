---
name: jig
description: Use when creating, editing, or building a ROS 2 package in this workspace - jig is the convention used here for declarative ROS 2 lifecycle nodes via interface.yaml + auto-generated session scaffolding, and also wraps launch-only / config-only / metapackages via the same `jig_auto_package()` macro. Triggers on "ROS node", "ROS package", interface.yaml, jig_auto_package, *Session subclasses, on_configure callbacks, or when files under nodes/<name>/ are involved.
---

# jig

Jig is the in-repo convention for ROS 2 lifecycle nodes. Source: `src/deps/jig/`. Examples: `src/deps/jig/jig_example/nodes/`. Full reference: `src/deps/jig/README.md`.

Jig turns one `interface.yaml` per node into a generated `Session` struct + lifecycle base class, so user code is just free functions that take `std::shared_ptr<Session>` (C++) or a `Session` dataclass (Python). The user never subclasses `LifecycleNode`.

## Package layout

```
my_package/
├── nodes/                           # optional - omit for launch/config/meta packages
│   └── my_node/
│       ├── interface.yaml           # required if node dir exists
│       ├── my_node.hpp              # C++ only
│       └── my_node.{cpp,py}         # language picked by extension
├── launch/                          # optional, auto-installed
├── config/                          # optional, auto-installed
├── interfaces/                      # optional, top-level shared interface.yaml files
├── CMakeLists.txt
└── package.xml
```

When present, `nodes/<dir_name>/` drives everything: the directory name (`my_node`) becomes the executable; `MyNode`/`MyNodeBase`/`MyNodeSession` are the generated PascalCase types. Plugin name is `${PROJECT_NAME}::${NodeName}`.

When `nodes/` is **absent**, `jig_auto_package()` is a thin wrapper around `ament_auto_package()` — code-gen, library/executable creation, and component registration are all skipped. It still installs `launch/`, `config/`, anything in `INSTALL_TO_SHARE`, and any top-level `interfaces/*.yaml`. Use this for launch-only, config-only, shared-interface, and metapackages.

## CMakeLists.txt

Node packages, launch-only packages, config-only packages, metapackages — all the same:
```cmake
cmake_minimum_required(VERSION 3.22)
project(my_package)
find_package(jig REQUIRED)
jig_auto_package()
```
Add `jig_auto_package(INSTALL_TO_SHARE maps rviz)` for extra share dirs. The macro adapts based on whether `nodes/` exists.

Interface-only packages (msg/srv/action definitions) — exactly:
```cmake
cmake_minimum_required(VERSION 3.22)
project(my_interfaces)
find_package(jig REQUIRED)
jig_auto_interface_package()
```
Drop `.msg`/`.srv`/`.action` into `msg/`, `srv/`, `action/`. Deps come from `package.xml`. The interface package.xml MUST include `<member_of_group>rosidl_interface_packages</member_of_group>`.

## package.xml essentials

`<depend>jig</depend>` and `<build_type>ament_cmake</build_type>` (even for Python-only nodes). For node packages also add `rclcpp` and/or `rclpy` and every msg/srv/action package referenced. Launch-only / config-only / metapackages just need `jig` plus whatever runtime packages they bring up.

## interface.yaml

`node:` is optional — name/package default from the build system. `${THIS_NODE}` / `${THIS_PACKAGE}` placeholders still work but are no longer needed.

Sections (all optional except you generally want at least one):

- `parameters:` — `generate_parameter_library` syntax. Common keys: `type`, `default_value`, `description`, `read_only`, `validation` (e.g. `gt<>: [0.0]`). `read_only: true` is required for any param referenced via `${param:...}` or `${for_each_param:...}`.
- `publishers:` / `subscribers:` — `topic`, `type` (e.g. `std_msgs/msg/String`), required `qos:`. QoS is **not** valid on services/actions/clients.
- Synchronised subscribers (sync groups): inside `subscribers:`, an item with `name`, `policy: approximate|exact`, `queue_size`, `max_interval` (required for approximate, forbidden for exact), and 2-9 `topics:`. Cannot use `${for_each_param:...}` in topic names.
- `services:` / `service_clients:` — `name`, `type`.
- `actions:` / `action_clients:` — `name`, `type`.
- `tf:` — flags `listener: true`, `broadcaster: true`, `static_broadcaster: true` (default false). Exposes `sn->tf_buffer`/`sn->tf_listener`/`sn->tf_broadcaster`/`sn->tf_static_broadcaster`.
- Any entity supports `manually_created: true` to document but skip generation, and `field_name:` to override the auto-derived session field name.

### QoS

Required: `history` (positive int for KEEP_LAST(n) or `"ALL"` for KEEP_ALL) and `reliability` (`RELIABLE` | `BEST_EFFORT`).
Optional: `durability` (`TRANSIENT_LOCAL`|`VOLATILE`), `deadline_ms`, `lifespan_ms`, `liveliness` (`AUTOMATIC`|`MANUAL_BY_TOPIC`), `lease_duration_ms`.

### Substitutions

- `${param:foo}` — interpolates a `read_only` param into a topic/service/action name OR a QoS field. Param type must match: int for numeric QoS fields, string for enum-like QoS fields. When used in a name, **`field_name:` becomes mandatory** (the substituted string can't form a C++ identifier).
- `${for_each_param:foo}` — `foo` must be a `read_only` `string_array`. Generates a `std::unordered_map<std::string, ...>` (C++) / `dict[str, ...]` (Python) of entities, one per element. `field_name:` is required. Only one `${for_each_param:...}` per name; can mix with `${param:...}`. Not allowed in sync group topic names.

## Session pattern — C++

`my_node.hpp`:
```cpp
#pragma once
#include <memory>
#include <my_package/my_node_interface.hpp>

namespace my_package::my_node {

struct Session : MyNodeSession<Session> {
    using MyNodeSession::MyNodeSession;
    int counter = 0;          // user state
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);
// optional: on_activate, on_deactivate, on_cleanup, on_shutdown
using MyNode = MyNodeBase<Session, on_configure /*, on_activate, ... */>;

} // namespace my_package::my_node
```

`my_node.cpp`:
```cpp
#include "my_node.hpp"
#include <jig/timer.hpp>          // for jig::create_timer
#include <jig/call_sync.hpp>      // for jig::call_sync, send_goal_sync, ...

namespace my_package::my_node {

void on_input(std::shared_ptr<Session> sn, std_msgs::msg::String::ConstSharedPtr msg) {
    sn->counter++;
    sn->publishers.output->publish(*msg);
}

CallbackReturn on_configure(std::shared_ptr<Session> sn) {
    sn->subscribers.input->set_callback(on_input);
    sn->services.svc->set_request_handler([](auto sn, auto req, auto resp){ /* ... */ });
    jig::create_timer(sn, 100ms, [](auto sn){ /* ... */ });
    return CallbackReturn::SUCCESS;
}

} // namespace my_package::my_node
```

Header include is `<package_name/<node_name>_interface.hpp>` (snake_case dir name, not the PascalCase type name).

Generated members on `sn` (the bits user code touches): `sn->node` (the lifecycle node), `sn->params.<name>`, `sn->publishers.<field>`, `sn->subscribers.<field>`, `sn->services.<field>`, `sn->service_clients.<field>`, `sn->actions.<field>`, `sn->action_clients.<field>`, `sn->tf_*` when enabled.

Lifecycle callbacks omitted from the `Base<...>` template default to `SUCCESS` (or no-op for `on_shutdown`). Order: configure → on_configure (entities already created) → activate → on_activate → entities activated; deactivate is the inverse.

## Session pattern — Python

```python
from dataclasses import dataclass
import jig
from jig import TransitionCallbackReturn
from my_package.my_node.interface import MyNodeSession, run

@dataclass
class MySession(MyNodeSession["MySession"]):
    counter: int = 0

def on_input(sn: MySession, msg):
    sn.counter += 1
    sn.publishers.output.publish(msg)

def on_configure(sn: MySession) -> TransitionCallbackReturn:
    sn.subscribers.input.set_callback(on_input)
    jig.create_timer(sn, sn.params.publish_rate_sec, lambda sn: ...)
    return TransitionCallbackReturn.SUCCESS

if __name__ == "__main__":
    run(MySession, on_configure,
        on_activate=..., on_deactivate=..., on_cleanup=..., on_shutdown=...)
```

Python import for the generated module is `<package>.<node_dir>.interface`. Service handlers must `return response`. Use `sn.logger` or `sn.node.get_logger()` for logging.

## Actions (C++)

`sn->actions.foo->set_options({.new_goals_replace_current_goal=bool, .goal_validator=[](const Goal&)->bool{...}});` then drive via a timer that calls `get_active_goal()`, `publish_feedback(...)`, `succeed(result)` / `abort(result)` / `cancel()`.

## Synchronous service/action calls (C++)

`#include <jig/call_sync.hpp>`. Helpers: `jig::call_sync<Srv>(client, req, 5s)` (returns `nullptr` on timeout), `jig::send_goal_sync<Action>(client, goal, {}, 5s)`, `jig::get_result_sync<Action>(client, handle, 5min)`, `jig::cancel_goal_sync<Action>(client, handle, 5s)`. Safe from `on_configure` because clients live on a dedicated background executor. **Don't call your own node's services this way** — the handler still needs the main executor.

## Behavioural defaults to remember

- **Single-threaded executor.** Session state has no locks; multi-threading is the caller's problem.
- **Autostart on by default** — node auto-runs configure→activate. Disable with `--ros-args -p autostart:=false` when using a lifecycle manager.
- **State heartbeat** on `~/state` at 10 Hz (`lifecycle_msgs/msg/State`, transient_local, 100 ms deadline). Useful as a watchdog topic.
- **Default QoS handlers** are auto-attached to every generated subscriber: deadline-missed or liveliness-lost → log error and `deactivate()` (only when ACTIVE). Setting your own `set_deadline_callback`/`set_liveliness_callback` replaces them. To make them fire, configure `deadline_ms`/`liveliness`+`lease_duration_ms` in QoS.
- **Intra-process comms ON by default for C++** — same-process Jig nodes pass shared pointers. Python unaffected.
- Subscriber/publisher event types: subscriber deadline = `QOSDeadlineRequestedInfo`; publisher deadline = `QOSDeadlineOfferedInfo`; subscriber liveliness = `QOSLivelinessChangedInfo`; publisher liveliness = `QOSLivelinessLostInfo`.

## Build & run

```bash
colcon build --packages-select my_package
source install/setup.bash
ros2 run my_package my_node
ros2 component standalone my_package my_package::MyNode    # C++ only
```

## Common pitfalls

- Forgetting `field_name:` when a topic/service name contains `${param:...}` or `${for_each_param:...}` — the build will fail.
- Adding `qos:` to a service/action/client — invalid; QoS is publisher/subscriber-only.
- Subclassing `LifecycleNode` directly — not the jig pattern. Subclass `<NodeName>Session` instead and pass `on_configure` as a template / kwarg.
- Calling a service hosted on the same node via `jig::call_sync` — deadlocks.
- Using `${for_each_param:...}` inside a sync group's topics — not supported (sync arity must be a compile-time constant).
- Param referenced from a substitution but not marked `read_only: true` — code generation rejects it.
- C++ header include must be `<package/<node_name>_interface.hpp>`; importing the PascalCase form fails.
- Python `interface` import: `from <package>.<node_dir>.interface import <NodeName>Session, run` — the `interface` submodule is required.
- Interface-only packages MUST have `<member_of_group>rosidl_interface_packages</member_of_group>` in `package.xml`.

## Where to look for examples

`src/deps/jig/jig_example/nodes/`:
- `echo_node/` (C++) — pubs/subs/services/clients/timer + `${param:...}` substitution.
- `py_echo_node/` (Python) — Python equivalent.
- `action_node/` (C++) — single-goal + replacement-mode action servers, action clients.
- `lifecycle_node/` (Python) — full lifecycle callbacks + advanced QoS.
- `for_each_node/` (Python) — `${for_each_param:...}` dynamic collections.
- `tf_node/` (Python) — TF2 listener/broadcaster/static_broadcaster.
- `sync_node/`, `py_sync_node/` — sync groups.

Full spec lives in `src/deps/jig/README.md` — go there when an edge case isn't covered above.
