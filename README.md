# syncai_robot_state (Rust)

The ROS 2 package `syncai_robot_state`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs` 0.8), plus a Docker environment for developing it on its own.

It is a Rust port of the C++ (rclcpp) `syncai_robot_state` in SyncAI-Robot-Workspace, and **its
external interface is deliberately identical**: the node name `syncai_robot_state`, the executable
`robot_state_node`, parameter names, topics, message types, QoS and the service it calls are all
the same, so it is a drop-in replacement and `syncai_backend` needs no changes. For behavioural
detail the C++ README is authoritative; [Differences from the C++ version](#differences-from-the-c-version)
below lists every place this port knowingly deviates.

**This repo is itself a single colcon package** (`package.xml` / `Cargo.toml` live at the root).
Like SyncAI-Robot-Driver-Manager, whose layout and conventions it follows, it is pulled into
SyncAI-Robot-Workspace with vcstool as `src/syncai_robot_state`.

## The node

One node that aggregates the robot's scattered status sources into a single
`syncai_common/RobotState` at **1 Hz as shipped** (10 Hz code default) on the relative topic
`robot_state`, so it lands on `<robot_id>/robot_state`.

```
    TF: map -> <robot_id>/base_link  ────┐   (syncai_localizer + syncai_lio_bridge)
    odom          (nav_msgs/Odometry) ───┤
    battery_state (BatteryState)  ───────┼──►  syncai_robot_state  ──► robot_state ──► syncai_backend
    wifi_status   (WifiStatus)    ───────┤          (1 Hz)                              (out of tree)
    motor_states  (MotorStates)   ───────┤                                                   │
    mode          (Int32MultiArray) ─────┘                                                   ▼
    get_mode      (service, polled) ─────┘                                    GET /api/v1/robot/state
```

One publisher, one timer, and syncai_backend as its only consumer — which has been out of tree
since 2026-09, so this topic is a cross-repository contract: widening a field is free, changing or
removing one is not.

It derives very little: a yaw extraction, a unit conversion, the localization-validity flag and one
latched threshold (low battery → `WARNING`). It holds no state beyond the latest sample of each
input and that one latch. **It only ever reports** — no threshold here commands the robot to do
anything.

### Interfaces

All names relative, so they inherit the `<robot_id>` namespace — except `/tf` and `/tf_static`,
which are absolute because TF is global in ROS 2. That is why the launch file prefixes the FRAME
name (`<robot_id>/base_link`) instead.

| Direction | Name | Type | QoS |
|---|---|---|---|
| Publish | `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, VOLATILE, KeepLast(1) |
| Subscribe | `odom` | `nav_msgs/Odometry` | SensorData |
| Subscribe | `battery_state` | `sensor_msgs/BatteryState` | SensorData |
| Subscribe | `wifi_status` | `syncai_common/WifiStatus` | BEST_EFFORT, VOLATILE, KeepLast(1) |
| Subscribe | `motor_states` | `syncai_common/MotorStates` | SensorData |
| Subscribe | `mode` | `std_msgs/Int32MultiArray` | RELIABLE, VOLATILE, KeepLast(10) |
| Subscribe | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | KeepLast(100), reliable; `/tf_static` also TRANSIENT_LOCAL |
| Service client | `get_mode` | `syncai_common/GetMode` | polled at `mode_poll_rate`, at most one request in flight |

`mode` is **the only RELIABLE endpoint in this node**, mirroring its publisher in
`syncai_driver_manager`: that topic is edge-triggered rather than periodic, so a dropped sample is
not made good by the next one. The flip side is that a publisher relaxed to best-effort would stop
matching this subscription **silently** — check with `ros2 topic info /<robot_id>/mode --verbose`.
The same trap points the other way at `robot_state`: a best-effort publisher cannot satisfy a
RELIABLE subscriber, so subscribing with the default rclcpp/rclpy QoS receives nothing at all.

### Parameters

| Parameter | Default | Set by the launch file |
|---|---|---|
| `robot_id` | `""` (yaml: `default_robot`) | `[system] robot_id` from the INI |
| `map` | `""` (yaml: `dp2f_full`) | `[map] map` from the INI, when present — a *path*, not a name |
| `global_frame` | `map` | — (stays unprefixed) |
| `base_frame` | `base_link` | `<robot_id>/base_link` |
| `transform_tolerance` | `0.1` | — (carried for parity; has no effect — see below) |
| `publish_rate` | `10.0` Hz | — (params file only; **the shipped file says `1.0`**) |
| `mode_poll_rate` | `1.0` Hz | — (params file only) |
| `low_battery_warn_percentage` | `20.0` % | — (params file only) |
| `low_battery_clear_percentage` | `25.0` % | — (params file only) |

All are **load-time only** and declared read-only, so `ros2 param set` is rejected rather than
silently ignored. A `low_battery_clear_percentage` at or below the warn value is rejected at
startup and both fall back to 20/25.

### The `state` derivation

Three of the six `RobotStatus` values are emitted, most-severe-first:

| `state` | Condition |
|---|---|
| `UNINITIALIZED` | `map → base_link` TF unavailable |
| `WARNING` | battery below the low-battery threshold (latched, with hysteresis) |
| `IDLE` | otherwise |

`UNINITIALIZED` outranks `WARNING` on purpose: "we don't know where the robot is" is the more
fundamental fact, and hiding it would leave `localization_status` reading as a pose when it is a
zero placeholder. `RUNNING` and `ERROR` are not derived yet; `CHARGING` cannot be, because
`syncai_driver_manager` hardcodes `BatteryState.power_supply_status` to `UNKNOWN`.

## Why Docker

ROS 2 has no official macOS support, and building ROS 2 + `rclrs` natively on macOS tends to get
stuck on dependencies. So the whole environment is packaged into a Linux container: **edit code on
the host, build and run inside the container.** The image carries ROS 2 Humble, Rust 1.85,
`colcon-cargo` / `colcon-ros-cargo`, and `rclrs` built from source into the underlay
`/opt/ros2_rust_underlay`, which is sourced automatically on entering the container.

## Quick start (VS Code Dev Container)

The only development environment is the Dev Container (`.devcontainer/`); there is no Makefile or
docker-compose. Once the package is in SyncAI-Robot-Workspace it is built with the Workspace's own
image, so this setup is only for developing this repo on its own.

1. Install the VS Code [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
2. Open this repo in VS Code and run **Dev Containers: Reopen in Container**
3. The first run builds the image (slow — message packages are compiled from source); later runs
   open instantly. After creation, `postCreateCommand` `vcs import`s the shared message package
   `syncai_common` and runs `colcon build` once
4. Run the node from a VS Code terminal (the namespace comes from `robot_id` in
   `~/robot_ws/config/system.ini`, falling back to `default_robot`):

```bash
ros2 launch syncai_robot_state robot_state.launch.py
```

Without VS Code, use the [devcontainer CLI](https://github.com/devcontainers/cli):
`devcontainer up --workspace-folder .`, then `docker exec -it syncai-robot-state bash -l`.

The container is named `syncai-robot-state` and its volumes are prefixed the same way, so it runs
side by side with the SyncAI-Robot-Driver-Manager container. Both default to `ROS_DOMAIN_ID=2` on
the host network, so the two nodes see each other's topics.

### Common commands (inside the container)

```bash
cd /workspace && colcon build --symlink-install --packages-select syncai_robot_state
cd /workspace/src/syncai_robot_state
cargo fmt                                                  # format (CI uses --check)
cargo clippy --target-dir /workspace/build/.clippy --all-targets
cargo test --target-dir /workspace/build/.clippy
```

`cargo` needs one `colcon build` first: `/workspace/.cargo/config.toml` is generated by
colcon-ros-cargo. Warnings from `/opt/ros2_rust_underlay/...` are rclrs's own; ignore them.

### CI

`.github/workflows/ci.yml` runs on every push to `main` / `dev` and on every pull request. One job
runs `cargo fmt --check` on the bare runner; the other builds this repo's Dev Container image with
buildx (layers cached between runs, so only the first run compiles rclrs from source), then runs
`.github/scripts/ci.sh` inside it: `vcs import`, `colcon build`, `cargo clippy -- -D warnings` and
`cargo test`. The script can be run by hand inside the Dev Container to reproduce CI exactly.

### Running it without the robot

```bash
ros2 run syncai_robot_state robot_state_node --ros-args -r __ns:=/default_robot \
    --params-file params/robot_state_params.yaml
```

Expect `localization_valid: false` and `state: 0` (`UNINITIALIZED`) until something publishes
`map -> base_link`, with every other field populated alongside. To exercise the rest — note the
`<robot_id>` namespace, and use a separate `ROS_DOMAIN_ID` so this never reaches the real robot:

```bash
export ROS_DOMAIN_ID=77

# A pose 1 m / 2 m out, rotated 90 degrees about Z
ros2 topic pub -r 10 /tf tf2_msgs/msg/TFMessage \
  "{transforms: [{header: {frame_id: map}, child_frame_id: base_link, transform: \
    {translation: {x: 1.0, y: 2.0, z: 0.0}, rotation: {z: 0.7071068, w: 0.7071068}}}]}"

# 19% -> state WARNING (3); 22% is inside the hysteresis band and stays WARNING;
# 26% clears it to IDLE (1); 0.0 simulates a corrupt BMS token and must change nothing
ros2 topic pub -r 5 /default_robot/battery_state sensor_msgs/msg/BatteryState \
  "{percentage: 0.19, present: true}"

ros2 topic echo --once --qos-reliability best_effort /default_robot/robot_state
```

`ros2 topic echo` without `--qos-reliability best_effort` receives nothing: the publisher is
best-effort.

## Project layout

```
.                           # = the ROS 2 package syncai_robot_state (build_type: ament_cargo)
├── package.xml
├── Cargo.toml / Cargo.lock
├── rustfmt.toml / clippy.toml / .editorconfig
├── launch/robot_state.launch.py    # reads robot_id (and [map] map) from system.ini
├── params/robot_state_params.yaml  # rates, frames, battery thresholds
├── src/
│   ├── main.rs
│   └── robot_state_node/
│       ├── mod.rs          # wiring: parameters → publisher → one worker → subscriptions + timers
│       ├── parameters.rs   # the nine load-time parameters, and Hz -> period
│       ├── state.rs        # the worker payload, sample intake, and the build-and-publish tick
│       ├── subscribers.rs  # the seven subscriptions and their QoS
│       ├── mode_poll.rs    # the get_mode client poll and its in-flight guard
│       ├── tf.rs           # the TF buffer and lookup that replace tf2_ros  (pure, unit-tested)
│       ├── health.rs       # the low-battery hysteresis latch                (pure, unit-tested)
│       └── wifi.rs         # WifiStatus -> the wifi_info JSON string         (pure, unit-tested)
│
│   # Only for developing this repo on its own; SyncAI-Robot-Workspace does not use these
├── interface.repos         # vcstool list: where the shared syncai_common messages come from
└── .devcontainer/          # Dockerfile + devcontainer.json
```

`tf.rs`, `health.rs` and `wifi.rs` are pure functions over plain Rust types — the ROS messages are
converted at the subscription boundary — so their tests need neither a ROS environment nor the
robot, the same split `protocol.rs` has in syncai_driver_manager.

## Differences from the C++ version

* **TF is looked up by this package, not by tf2_ros.** rclrs has no `tf2_ros` binding, so the node
  subscribes `/tf` and `/tf_static` itself and walks the tree in `src/robot_state_node/tf.rs`. It
  keeps only the newest transform per child frame, with no time history, no interpolation and no
  cache expiry. The C++ looks up at `tf2::TimePointZero`, which resolves to the newest sample
  anyway, so nothing is lost on the lookup this node actually performs.
  * **Consequence, shared with the C++ version but not stated there:** once `map -> base_link` has
    been seen ONCE, `localization_valid` stays true forever, even if the localizer dies and the
    pose freezes. tf2 would not catch it either — it never drops the newest entry for a frame.
  * **`transform_tolerance` therefore has no effect**, in either version. In the C++ the
    non-blocking `canTransform()` gate means it only ever applied to a race between the check and
    the call; here the lookup is a few hash lookups against an in-memory map, so that race does not
    exist. The parameter is kept so the params file and `ros2 param list` stay identical.
* **Parameters are declared read-only**, so `ros2 param set` is rejected. The C++ reads each
  parameter once and has no `add_on_set_parameters_callback`, so a set there appears to succeed and
  changes nothing. Same effective behaviour, made visible.
* **One worker instead of two callback groups and a mutex.** The seven subscriptions and both
  timers run on a single rclrs `Worker`, whose callbacks never overlap — which is exactly the
  protection the C++ `mutex_` provides, with no lock to take. The C++ needs a second
  `MutuallyExclusive` group because `tf2_ros::Buffer::transform()` blocks for the whole
  `transform_tolerance` when the transform is absent; nothing in this port blocks. The one
  exception is the `get_mode` response callback, which the executor runs on its own task pool and
  which reaches a small `Mutex` in `mode_poll.rs`.
* **The params YAML key is `/**`, not `/**/syncai_robot_state`.** rclrs only matches keys that are
  exactly `/**` or the node's full name (`/<robot_id>/syncai_robot_state`) and **does not expand
  wildcards**; `/**/syncai_robot_state` raises no error, it just silently falls back to the code
  defaults for everything — including a 10 Hz publish rate instead of the intended 1 Hz.
* **An abandoned `get_mode` request is not pruned.** rclrs has no `prune_pending_requests`, so when
  a reply is lost mid-flight the in-flight guard still releases after 5 polls and a fresh request
  goes out, but the dead callback stays in the client's request board. While sys_manager is simply
  down, the readiness check stops us sending at all, so this costs one closure per lost reply.
  Each request carries a generation number, so should the abandoned reply arrive after all it is
  recognised and ignored rather than releasing the guard under its replacement or overwriting the
  fresher answer.
* **`wifi_info` is built with serde_json rather than nlohmann/json**, and produces the same bytes:
  serde_json's default map is a `BTreeMap`, so keys come out alphabetically ordered exactly as
  nlohmann's `std::map`-backed object does, and no sample still dumps to the literal string `null`.
* **There is no SIGINT handler.** rclrs does not handle signals, so Ctrl-C / `ros2 launch` shutdown
  terminates the process with the default action and `Drop` does not run. A process backgrounded
  with `&` from a non-interactive shell script ignores SIGINT; stop it with SIGTERM in scripts.

## rclrs notes

* **The executor comes before the node.** `Context::default_from_env()?.create_basic_executor()`,
  then `executor.create_node(..)`, then `executor.spin(..)`. There is no equivalent of the C++
  main's "hold the node in a named variable" trap.
* **A Worker is rclrs's callback group.** `node.create_worker(payload)` gives callbacks of the
  shape `FnMut(&mut Payload, Msg)`; callbacks under one worker run one at a time, while different
  workers run in parallel on their own threads. Callbacks created from the node itself all queue on
  the executor's single thread.
* **Message types come from `ros-env`** (`use ros_env::nav_msgs::msg::Odometry;`). Message packages
  go in `<depend>` in `package.xml`, **not** in `Cargo.toml`; without the `<depend>`, colcon does
  not put the package on `AMENT_PREFIX_PATH` and `ros_env::<pkg>` does not exist.
* **The underlay is built with `CARGO_RESOLVER_INCOMPATIBLE_RUST_VERSIONS=fallback`.** The image
  pins Rust 1.85, which is rclrs's own `rust-version`, but rclrs commits no lock file and its
  dependencies drift: a fresh resolve already picks a `uuid` that needs rustc 1.89 and the image
  build fails with "rustc 1.85.0 is not supported". The fallback policy makes cargo prefer the
  newest dependency version whose `rust-version` fits the toolchain, so a rebuild on a new machine
  (or in CI) resolves the same way the pin intends.
* **`tf2_msgs` is the one message package not rebuilt by the underlay.** ros2_rust's `.repos` lists
  `common_interfaces` and `rcl_interfaces`; `tf2_msgs` lives in `ros2/geometry2`. It does not need
  to be rebuilt — the official `ros-humble-tf2-msgs` deb already ships generated Rust bindings at
  `share/tf2_msgs/rust`, which is what this package links against. If a future base image stops
  shipping them, add geometry2's `tf2_msgs` to the underlay's `.repos`.
* **`ros-env` versions**: this package uses `ros-env` 0.2 while rclrs uses 0.3 internally, and their
  message types are incompatible. Do not use rclrs helpers that return message types (e.g.
  `Time::to_ros_msg`); build them yourself from `Time::nsec`.

## The shared message package syncai_common

The `msg` / `srv` / `action` definitions live in
[SyncAI-Robot-Interface](https://github.com/chungweeeei/SyncAI-Robot-Interface) (colcon package name
`syncai_common`), shared by the whole syncai stack, and are pulled into the container workspace's
`src/` with [vcstool](https://github.com/dirk-thomas/vcstool) rather than a git submodule. It is
pinned to the `dev` branch. To update it by hand, inside the container:

```bash
cd /workspace
vcs import < src/syncai_robot_state/interface.repos           # first time
vcs import --force < src/syncai_robot_state/interface.repos   # discard local changes / change version
```

## Known limits (inherited from the C++ version)

* **No battery-sample staleness detection.** The last sample never expires, so if
  `syncai_driver_manager` dies the latch freezes on its last verdict: died at 15% → `WARNING`
  forever (errs safe), died at 80% → `IDLE` forever (misleading).
* **`WARNING` carries no reason.** A second `WARNING` condition will need a reason field or bitmask.
* **`low_level_mode` has no freshness information at all.** `0 / 0` before the first sample is
  indistinguishable from a genuine "PPO / Stand", and a frozen value is equally consistent with a
  dead driver, a controller that stopped sending the MODE_STATE section, and nothing having
  changed. `motor_status.timestamp` advancing is the nearest available proxy for
  "syncai_driver_manager is alive".
* **`motor_status.timestamp` is seconds here but nanoseconds on the `motor_states` topic.** This
  node scales it on the way in. The same message type means two different units depending on where
  you read it.

## References

* [ros2_rust](https://github.com/ros2-rust/ros2_rust)
* [rclrs examples](https://github.com/ros2-rust/examples/tree/main/rclrs)
* [rclrs on docs.rs](https://docs.rs/rclrs)
