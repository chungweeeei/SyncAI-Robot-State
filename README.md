# syncai_robot_state (Rust)

The ROS 2 package `syncai_robot_state`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs` 0.8), plus a Docker environment for developing it on its own.

It is a Rust port of the C++ (rclcpp) `syncai_robot_state` in SyncAI-Robot-Workspace, and **its
external interface is deliberately identical**: the node name `syncai_robot_state`, the executable
`robot_state_node`, parameter names, topics, message types, QoS and the service it calls are all
the same, so it is a drop-in replacement — **of the reporting half**. Since 2026-11 this node
also owns the robot's state machine and serves `set_robot_state` / `begin_task` / `end_task`,
which the C++ version never had ([The state machine](#the-state-machine)). In the reporting half
there are two deliberate exceptions: the
`safety_locked` subscription, which fills `low_level_mode.safety_state` — a field the backend has
to rebuild against SyncAI-Robot-Interface to read, though nothing it reads today changed — and
`localization_status.position.z`, which is base_link's real height read through the lidar rather
than the planar chain's 0 (two more frame parameters). For behavioural detail the C++ README is
authoritative; [Differences from the C++ version](#differences-from-the-c-version) below lists
every place this port knowingly deviates, both exceptions included.

**This repo is itself a single colcon package** (`package.xml` / `Cargo.toml` live at the root).
Like SyncAI-Robot-Driver-Manager, whose layout and conventions it follows, it is pulled into
SyncAI-Robot-Workspace with vcstool as `src/syncai_robot_state`.

## The node

One node that aggregates the robot's scattered status sources into a single
`syncai_common/RobotState` at **1 Hz as shipped** (10 Hz code default) on the relative topic
`robot_state`, so it lands on `<robot_id>/robot_state`.

```
    TF: map -> <robot_id>/base_link  ────┐   (syncai_localizer + syncai_lio_bridge)
    TF: map -> <robot_id>/pointlio_body ─┤   (z only: syncai_localizer + Point-LIO, via lidar_top_imu)
    odom          (nav_msgs/Odometry) ───┤
    battery_state (BatteryState)  ───────┼──►  syncai_robot_state  ──► robot_state ──► syncai_backend
    wifi_status   (WifiStatus)    ───────┤          (1 Hz)                              (out of tree)
    motor_states_low_rate (20 Hz) ───────┤                                                   │
    mode          (Int32MultiArray) ─────┤                                                   │
    safety_locked (Bool, latched) ───────┤                                                   ▼
    get_mode      (service, polled) ─────┤                                    GET /api/v1/robot/state
    localization_status (latched) ───────┘   (syncai_localizer: map, converged, pose_epoch)

    syncai_backend ──begin_task / end_task──►  syncai_robot_state   (every task run, a schedule's too)
    syncai_backend ──set_robot_state────────►  syncai_robot_state   (the operator's Confirm control)
```

One publisher, one timer, three services, and syncai_backend as the only consumer and caller —
which has been out of tree since 2026-09, so the topic and the services are a cross-repository
contract: widening a field is free, changing or removing one is not.

Most of the message is a straight copy: a yaw extraction, the height through the lidar, a unit
conversion, the TF-presence flag, one latched threshold (low battery → `WARNING`) and the
battery's charge status (→ `CHARGING`). **`state` is the exception since 2026-11**: it rolls up a
state machine this node owns — whether an operator has confirmed the position, whether a task is
running, whether a failure is latched — and through `begin_task` that is a decision the backend
obeys. It is the one thing here that is not a report. Still no threshold commands the robot to do
anything; refusing a task is the whole of it.

### Interfaces

All names relative, so they inherit the `<robot_id>` namespace — except `/tf` and `/tf_static`,
which are absolute because TF is global in ROS 2. That is why the launch file prefixes the FRAME
name (`<robot_id>/base_link`) instead.

| Direction | Name | Type | QoS |
|---|---|---|---|
| Publish | `robot_state` | `syncai_common/RobotState` | BEST_EFFORT, VOLATILE, KeepLast(1) |
| Subscribe | `odom_topic`: `odom` | `nav_msgs/Odometry` | SensorData |
| Subscribe | `battery_state_topic`: `battery_state` | `sensor_msgs/BatteryState` | SensorData |
| Subscribe | `wifi_status_topic`: `wifi_status` | `syncai_common/WifiStatus` | BEST_EFFORT, VOLATILE, KeepLast(1) |
| Subscribe | `motor_states_topic`: `motor_states_low_rate` | `syncai_common/MotorStates` | SensorData. driver_manager's 20 Hz copy of `motor_states` (~200 Hz), since 2026-11: only the latest sample is used |
| Subscribe | `mode_topic`: `mode` | `std_msgs/Int32MultiArray` | RELIABLE, VOLATILE, KeepLast(10) |
| Subscribe | `safety_locked_topic`: `safety_locked` | `std_msgs/Bool` | RELIABLE, TRANSIENT_LOCAL, KeepLast(1) |
| Subscribe | `localization_status` | `syncai_common/LocalizationStatus` | RELIABLE, TRANSIENT_LOCAL, KeepLast(1). The localizer's facts, published on a change only (2026-11) |
| Subscribe | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | KeepLast(100), reliable; `/tf_static` also TRANSIENT_LOCAL |
| Service | `set_robot_state` | `syncai_common/SetRobotState` | The operator's switch: confirm / withdraw the position, release an error |
| Service | `begin_task` | `syncai_common/BeginTask` | May this task start? An acceptance is the transition to `RUNNING` |
| Service | `end_task` | `syncai_common/EndTask` | How an accepted task ended; `FAILED` latches `ERROR` |
| Service client | `get_mode` | `syncai_common/GetMode` | polled at `mode_poll_rate`, at most one request in flight |

**Six topic names are parameters** (2026-11-24), set per robot model in
`params/models/<robot_model>.yaml` (see "Parameters"); the names above are the G23's, and the code
defaults. An empty name is **not subscribed**, logged once at startup
(`[Subscriptions] mode: no topic set for this robot model, not subscribed`), and the RobotState
fields it feeds stay at their defaults, as they do while a publisher is missing. The QoS of each
is fixed in code whatever the name. `localization_status` is not one of them: it is the
localizer's, in this stack on every model, and keeps its fixed relative name.

`mode`, `safety_locked` and `localization_status` are **the only RELIABLE subscriptions in this
node**, mirroring their publishers (`syncai_driver_manager`, and `syncai_localizer` for the last): both topics are edge-triggered rather than periodic, so a
dropped sample is not made good by the next one. The flip side is that a publisher relaxed to
best-effort would stop matching these subscriptions **silently** — check with
`ros2 topic info /<robot_id>/mode --verbose` (and the same for `safety_locked`).

`safety_locked` is additionally **TRANSIENT_LOCAL**, again mirroring its publisher, which is
latched: the driver publishes the released state once at startup and then only on a change. This
node comes up after the driver, so a volatile reader would never hear that startup sample and would
report `false` until the lock next moved. Its value lands in `low_level_mode.safety_state`; a `mode`
sample writes only `policy_state` / `motion_state`, so the two topics cannot overwrite each other.
Durability has the same silent failure as reliability: a transient-local reader matches only a
transient-local writer, so a driver relaxed to VOLATILE would leave `safety_state` at `false` with
no error anywhere. Change the two sides together.

`localization_status` is latched for the same reason and has the same trap: it is published on a
change only, this node may start after the last one, and against a localizer relaxed to VOLATILE
the robot would read "not converged" for good — the position could never be confirmed.

The same trap points the other way at `robot_state`: a best-effort publisher cannot satisfy a
RELIABLE subscriber, so subscribing with the default rclcpp/rclpy QoS receives nothing at all.

### Parameters

| Parameter | Default | Set by the launch file |
|---|---|---|
| `robot_id` | `""` (yaml: `default_robot`) | `[system] robot_id` from the INI |
| `map` | `""` | `[map] name` from the INI (an older INI's `[map] map` path → its directory name; no map named → `""`) |
| `global_frame` | `map` | — (stays unprefixed) |
| `base_frame` | `base_link` | `<robot_id>/base_link` |
| `body_frame` | `pointlio_body` | `<robot_id>/pointlio_body` (not in the C++ version) |
| `lidar_frame` | `lidar_top_imu` | `<robot_id>/` + the model file's value (G23 `lidar_top_imu`, M20 `lidar_link`; not in the C++ version; `lidar_top` until 2026-10) |
| `odom_topic` … `safety_locked_topic` | the G23's names (table above) | — (the model file) |
| `transform_tolerance` | `0.1` | — (carried for parity; has no effect — see below) |
| `publish_rate` | `10.0` Hz | — (params file only; **the shipped file says `1.0`**) |
| `mode_poll_rate` | `1.0` Hz | — (params file only) |
| `low_battery_warn_percentage` | `20.0` % | — (params file only) |
| `low_battery_clear_percentage` | `25.0` % | — (params file only) |

**Per robot model** (2026-11-24, the layout `syncai_driver_manager` has): the launch reads
`[system] robot_model` from the INI (`g23` when absent) and loads `params/models/<robot_model>.yaml`
after `robot_state_params.yaml`, all three layers under `/**`, so a later one wins key by key. A
model's file sets the six topics and `lidar_frame` (unprefixed; the launch adds `<robot_id>/`), and
the launch **refuses** a model with no file or one that leaves one of them out: the code defaults
are the G23's. `g23.yaml` holds the names hard-coded until then. `m20.yaml` (DEEP Robotics Lynx M20)
keeps the same names — on robot01 only `wifi_status`, `safety_locked` (and `odom`, once
`syncai_lio_bridge` runs) have a publisher; the vendor's battery and joint data are drdds types this
node does not read — leaves `mode_topic` empty (the vendor's gait state is `drdds/MotionState`, only
in the integrator's drdds), and sets `lidar_frame: lidar_link`, the vendor's merged-cloud frame at
`base_link`'s origin, 0.0435 m below the M20's IMU that Point-LIO's body is: the reported height is
that much off, `imu` is the exact link. A new model's file needs a colcon build.

All are **load-time only** and declared read-only, so `ros2 param set` is rejected rather than
silently ignored. A `low_battery_clear_percentage` at or below the warn value is rejected at
startup and both fall back to 20/25.

### The state machine

Until 2026-11 `state` was derived from the TF tree and the battery, and `UNINITIALIZED` meant only
"`map → base_link` has never been seen". The localizer broadcasts that transform from Point-LIO's
first odometry on — the identity before anything has converged, the previous map's correction
after a map switch — so the state read `IDLE` a few seconds after boot and stayed `IDLE` through a
switch, and a task (a scheduled one above all, which nobody is watching) drove off from a pose no
one had looked at. `RUNNING` and `ERROR` were declared and never emitted.

Now this node owns the machine (`lifecycle.rs`) and is the one judge of whether a task may start:

```
start ──► UNINITIALIZED ──set_robot_state(IDLE), needs converged──► IDLE
IDLE ──begin_task accepted──► RUNNING ──end_task(COMPLETED / CANCELED)──► IDLE
RUNNING ──end_task(FAILED)──► ERROR ──set_robot_state──► IDLE or UNINITIALIZED
IDLE ──another pose_epoch / set_robot_state(UNINITIALIZED)──► UNINITIALIZED
```

All six `RobotStatus` values are emitted, most-severe-first (`health::derive_status`):

| `state` | Condition |
|---|---|
| `ERROR` | a task ended `FAILED` (`end_task`). **Latched**: only `set_robot_state` releases it. Refuses **every** task |
| `UNINITIALIZED` | no operator has confirmed the localizer's current estimate, or there is no `map → base_link` to report. Refuses a task with a MOVE / PATROL / CHARGE step |
| `RUNNING` | a task `begin_task` accepted and `end_task` has not closed. One at a time |
| `CHARGING` | latched: engaged by a `battery_state` reading `POWER_SUPPLY_STATUS_CHARGING`, released only by one reading `DISCHARGING` |
| `WARNING` | battery below the low-battery threshold (latched, with hysteresis) |
| `IDLE` | otherwise: confirmed, nothing running |

`state_message` says why in one sentence written for an operator (what the robot is waiting for,
that a task runs, what failed), and `localization_converged` says the position *can* be confirmed.

**The order is part of the contract.** `UNINITIALIZED` sits above `RUNNING` so that "neither
`ERROR` nor `UNINITIALIZED`" reads as "the position is confirmed" with no field for it; the price
is that a posture task run on an unconfirmed robot reads `UNINITIALIZED` while it runs. `ERROR`
sits above everything because it is the one state that stops every task and that only a person
ends: an operator shown `UNINITIALIZED` would confirm the position and still have a robot that
refuses.

**A confirmation is a remembered `pose_epoch`, not a flag.** `set_robot_state(IDLE)` stores the
epoch `localization_status` carried at that moment, and the robot is confirmed for exactly as long
as the localizer still reports that value. A map switch, a new initial pose, an auto-localize and
a restarted localizer all change it, so each withdraws the confirmation without an event this node
could miss or see out of order. A guess that is queued and converges between two samples —
`converged` never reads false — is withdrawn the same way, which is what a flag cleared on
`converged: false` gets wrong. A converged boot guess confirms nothing: it only makes the position
confirmable.

**`begin_task`** is asked by the backend as the first activity of every task run, a run a schedule
started by itself included. Refusals, in the order they are judged:

| Code | When | Applies to |
|---|---|---|
| `ERROR_LATCHED` | a failure is latched | every task |
| `NOT_INITIALIZED` | the position is not confirmed | a task with `needs_localization` |
| `BUSY` | another `task_id` is running | every task |

A refusal changes nothing — in particular it never latches `ERROR`. An acceptance *is* the
transition, so two runs cannot both be let through and there is no `robot_state` sample to read a
second too late. It is idempotent per `task_id` (judged before anything else): a caller that lost
the first answer and retries would otherwise read `BUSY` against itself and, told no, never send
the `end_task` that lets the robot go.

**`end_task`** is matched by `task_id` and a no-op otherwise, which makes a repeated report
harmless and keeps a run this node forgot (it restarted mid-task) from latching an `ERROR` its log
cannot explain. `COMPLETED` and `CANCELED` change nothing but the task: a robot that stood up
unconfirmed is unconfirmed afterwards.

**`set_robot_state`** is the only way into "confirmed" and the only way out of `ERROR`. `IDLE` is
refused `NOT_CONVERGED` without a converged estimate (no localizer heard from — every moment of
the mapping session — no map, or a guess that has not landed), and a refusal releases nothing.
Either target also drops a task still counted as running. That is the recovery for a workflow
terminated from outside, whose `end_task` never comes, and it is why this service does **not**
refuse while a task runs: the backend does, from its own job list.

**In memory only.** A restart of this node is `UNINITIALIZED` with no error and no task. That is
the wanted behaviour for the restarts that happen — a boot, a mode switch and `restart_mode` all
bring the localizer up afresh beside it — and the safe one for a crash: it can forget a
confirmation, never invent one. The mapping session runs no localizer, so the robot is
`UNINITIALIZED` for the whole run and still takes a task that needs no position.

**ERROR has more than one cause by design.** Today the one cause is a failed task. A fault
reported by `syncai_driver_manager` is to be the second; it has no such signal yet. When it does,
it is a `FaultSource` variant, a subscription and one call — the latch, the refusals and the
release do not change.

Every transition and every refusal is logged with the values that decided it (the task id,
`pose_epoch`, the map, `converged`, whether a pose is available), and each transition publishes a
`robot_state` at once instead of waiting for the 1 Hz tick.

The battery states are unchanged. `CHARGING` outranks `WARNING`: a low pack on its charger is what
is being fixed. `CHARGING` is syncai_driver_manager's: it derives `power_supply_status` from the
sign of the pack's **net** current with a 0.1 A deadband, so it means the charger delivers more
than the robot draws. A nearly full pack's current tapers into that deadband (`NOT_CHARGING`) with
the robot still on its charger, so the state is latched (`health::ChargingLatch`): it holds
through `NOT_CHARGING` and drops on the first `DISCHARGING`, which a robot taken off the charger
reads as soon as it draws its own load. Not a percentage rule: this node does not know the robot
is on a charger, and a full robot idling elsewhere must not read `CHARGING`. No freshness rule, as
for every other input: a driver that stops publishing leaves the last status in place.

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

Expect `localization_valid: false` until something publishes `map -> base_link`, and `state: 0`
(`UNINITIALIZED`) for as long as nothing has confirmed the position — which a fake TF never does —
with every other field populated alongside. A fake `map -> base_link` alone
reports its own z and a throttled "Height through the lidar unavailable" warning, since there is no
`pointlio_body` / `lidar_top_imu` behind it. To exercise the rest — note the
`<robot_id>` namespace, and use a separate `ROS_DOMAIN_ID` so this never reaches the real robot:

```bash
export ROS_DOMAIN_ID=77

# A pose 1 m / 2 m out, rotated 90 degrees about Z
ros2 topic pub -r 10 /tf tf2_msgs/msg/TFMessage \
  "{transforms: [{header: {frame_id: map}, child_frame_id: base_link, transform: \
    {translation: {x: 1.0, y: 2.0, z: 0.0}, rotation: {z: 0.7071068, w: 0.7071068}}}]}"

# Stand in for the localizer: a converged estimate, latched, as it publishes it
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /default_robot/localization_status syncai_common/msg/LocalizationStatus \
  "{map: office, converged: true, pose_epoch: 7}"

# Still UNINITIALIZED (0): a converged estimate confirms nothing. Confirm it -> IDLE (1)
ros2 service call /default_robot/set_robot_state syncai_common/srv/SetRobotState "{state: 1}"

# RUNNING (2), then ERROR (4) and every begin_task refused, then released
ros2 service call /default_robot/begin_task syncai_common/srv/BeginTask \
  "{task_id: t1, needs_localization: true}"
ros2 service call /default_robot/end_task syncai_common/srv/EndTask \
  "{task_id: t1, outcome: 2, reason: 'the goal is occupied'}"
ros2 service call /default_robot/set_robot_state syncai_common/srv/SetRobotState "{state: 1}"

# Another pose_epoch (a new guess, a map switch) -> UNINITIALIZED again
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /default_robot/localization_status syncai_common/msg/LocalizationStatus \
  "{map: office, converged: true, pose_epoch: 8}"

# With the position confirmed: 19% -> state WARNING (3); 22% is inside the hysteresis band and
# stays WARNING; 26% clears it to IDLE (1); 0.0 simulates a corrupt BMS token and must change nothing
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
├── launch/robot_state.launch.py    # reads robot_id (and [map] name) from system.ini
├── params/robot_state_params.yaml  # rates, frames, battery thresholds (every model)
├── params/models/<model>.yaml      # g23 / m20: input topics, lidar_frame
├── src/
│   ├── main.rs
│   └── robot_state_node/
│       ├── mod.rs          # wiring: parameters → publisher → one worker → subscriptions + timers
│       ├── parameters.rs   # the eleven load-time parameters, and Hz -> period
│       ├── state.rs        # the worker payload, sample intake, the service handlers, and the build-and-publish tick
│       ├── subscribers.rs  # the nine subscriptions and their QoS
│       ├── services.rs     # set_robot_state / begin_task / end_task, on the same worker
│       ├── lifecycle.rs    # the robot's state machine                       (pure, unit-tested)
│       ├── mode_poll.rs    # the get_mode client poll and its in-flight guard
│       ├── tf.rs           # the TF buffer and lookup that replace tf2_ros  (pure, unit-tested)
│       ├── health.rs       # the battery latches and the `state` rollup      (pure, unit-tested)
│       └── wifi.rs         # WifiStatus -> the wifi_info JSON string         (pure, unit-tested)
│
│   # Only for developing this repo on its own; SyncAI-Robot-Workspace does not use these
├── interface.repos         # vcstool list: where the shared syncai_common messages come from
└── .devcontainer/          # Dockerfile + devcontainer.json
```

`tf.rs`, `health.rs`, `lifecycle.rs` and `wifi.rs` are pure functions over plain Rust types — the ROS messages are
converted at the subscription boundary — so their tests need neither a ROS environment nor the
robot, the same split `protocol.rs` has in syncai_driver_manager.

## Differences from the C++ version

* **`state` can be `CHARGING`** (2026-11-18), from `battery_state.power_supply_status`, outranking
  `WARNING`. The C++ version never emits it: when it was written syncai_driver_manager hardcoded
  that status to `UNKNOWN`; the Rust driver_manager derives it from the current's sign. Same
  field, same constant (`RobotStatus.CHARGING` = 5, declared since the renumbering), so nothing
  needs a rebuild.
* **`safety_locked` is subscribed, and `low_level_mode.safety_state` is filled from it.** The one
  deliberate widening of the interface: the C++ version has neither, because the topic itself is a
  Rust-only addition in syncai_driver_manager. It needs `syncai_common` at or after
  SyncAI-Robot-Interface `306f04a` (PR #6, merged into `dev`), which added the field — so unlike
  everything else in this list, the backend does have to rebuild against the new interface to
  read it.
* **`localization_status.position.z` is base_link's real height, not the z of
  `map -> base_link`.** The second deliberate deviation. syncai_lio_bridge broadcasts
  `odom -> base_link` and `map -> odom` planar for the 2D nav stack, so the chain the C++ reads
  reports z = 0 on every storey, and an upstairs robot reads as standing on the ground floor. The
  height is read instead through the one chain that still climbs, Point-LIO's
  `map -> <robot_id>/pointlio_odom -> <robot_id>/pointlio_body` (the body frame is the lidar's
  built-in IMU), closed onto the base with the URDF's static
  `<robot_id>/base_link -> <robot_id>/lidar_top_imu`:
  `map <- base = (map <- body) · (base <- lidar)⁻¹`, in full 3D, because once the base pitches on
  a stair the lidar's 0.195 m forward offset moves its height too. This is the same compose the
  backend's telemetry does in `TelemetrySubscriber._lio_odom_cb`. Two parameters name the frames,
  `body_frame` and `lidar_frame`, prefixed with `<robot_id>/` by the launch file like `base_frame`.
  `lidar_frame` was `lidar_top` until 2026-10, when syncai_bringup's URDF split the lidar
  (`lidar_top`, the frame its cloud is stamped with) from its IMU (`lidar_top_imu`, `-t_il` below
  it); closing the IMU's chain through the lidar put the height ~4.5 cm off.
  * **x / y / yaw and `localization_valid` are unchanged**: they still come from
    `map -> base_link`, which is what the nav stack drives by. Only a failed planar lookup withholds
    the pose.
  * **While the lidar chain is missing** (Point-LIO not up, a wrong frame name) the planar z is
    reported, as the C++ does, with a throttled warning — never a skipped tick.
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
* **One worker instead of two callback groups and a mutex.** The nine subscriptions, the three services and both
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
  (`ERROR` and `UNINITIALIZED` do have one since 2026-11: `state_message`.)
* **`localization_status` has no liveness either.** It is latched and published on a change only,
  so a localizer that died after reporting `converged: true` leaves the position confirmable, and
  a confirmed robot confirmed. Going lost does not withdraw a confirmation: the localizer's health
  state is not on the topic, on purpose (its thresholds are not replay-verified).
* **The gate is only as wide as its callers.** `begin_task` stops a task the backend runs. A goal
  sent straight to `navigate_to_goal` (rviz, the `ros2` CLI) and the console's teleop are accepted
  in any state — nothing in the navigation stack asks this node.
* **A task counted as running is never timed out.** If its `end_task` never comes the robot
  answers `BUSY` until an operator's `set_robot_state`; there is no heartbeat from the backend to
  age it by.
* **`ERROR` does not survive a restart of this node**, which comes up `UNINITIALIZED` with nothing
  latched: a task that needs no position is then accepted again.
* **`low_level_mode` has no freshness information at all.** `0 / 0 / false` before the first sample
  is indistinguishable from a genuine "PPO / Stand, lock released", and a frozen value is equally
  consistent with a dead driver, a controller that stopped sending the MODE_STATE section, and
  nothing having changed. `motor_status.timestamp` advancing is the nearest available proxy for
  "syncai_driver_manager is alive".
* **`motor_status.timestamp` is seconds here but nanoseconds on the `motor_states` topic.** This
  node scales it on the way in. The same message type means two different units depending on where
  you read it.

## References

* [ros2_rust](https://github.com/ros2-rust/ros2_rust)
* [rclrs examples](https://github.com/ros2-rust/examples/tree/main/rclrs)
* [rclrs on docs.rs](https://docs.rs/rclrs)
