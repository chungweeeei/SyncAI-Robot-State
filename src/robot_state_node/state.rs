//! The worker payload and the build-and-publish tick.
//!
//! Everything here runs on the node's single [`Worker`], so the five sample caches, the TF buffer,
//! the health latches and the state machine need no locking at all — the worker is rclrs's equivalent of an rclcpp
//! `MutuallyExclusive` callback group and guarantees its callbacks never overlap. That replaces
//! the C++ version's `mutex_`; see the threading note in `mod.rs` for why one worker is enough
//! here when the C++ needed two callback groups.

use rclrs::*;
use ros_env::syncai_common::msg::{
    MotorStates, RobotLowLevelMode, RobotPose, RobotState, RobotStatus,
};
use ros_env::syncai_common::srv::{
    BeginTask_Request, BeginTask_Response, EndTask_Request, EndTask_Response,
    SetRobotState_Request, SetRobotState_Response,
};
use std::time::Duration;

use super::health::{
    self, ChargeReading, ChargingLatch, LatchChange, LowBatteryLatch, Rollup, Status,
};
use super::lifecycle::{
    BeginRefusal, Begun, Ended, Lifecycle, Localization, Outcome, Released, Target,
};
use super::mode_poll::ModePoll;
use super::parameters::Config;
use super::tf::{InvalidTransform, ReportedPose, TfBuffer, Transform};
use super::wifi::{self, WifiInfo};

// The C++ throttles these at 2 s (TF) and 5 s (everything else); matched so the log reads the same
const TF_LOG_THROTTLE: Duration = Duration::from_secs(2);
const SAMPLE_LOG_THROTTLE: Duration = Duration::from_secs(5);

/// Everything the node holds: the latest sample of each input, the TF tree, the two battery
/// latches, the state machine, and the handles needed to publish.
///
/// Until 2026-11 the node held no state beyond the latest sample and one latch, and only ever
/// reported. It now also owns [`Lifecycle`] — the operator's confirmation of the position, the
/// running task, a latched failure — and answers whether a task may start. That is the one place
/// it decides anything; `low_level_mode` is still carried through untouched, two integers and a
/// bool, no verdict, no freshness field.
pub struct RobotStateData {
    config: Config,
    publisher: Publisher<RobotState>,
    clock: Clock,
    logger: Logger,
    mode_poll: ModePoll,

    tf: TfBuffer,
    latch: LowBatteryLatch,
    /// The robot's state machine. Advanced by `localization_status` and by the three services;
    /// read by the rollup in [`Self::build_state`]. `lifecycle.rs` has what it is and why it
    /// lives in this node.
    lifecycle: Lifecycle,

    /// Forward speed from `odom`. The whole Odometry message is not kept: only `linear.x` is ever
    /// read, so there is nothing else to hold alive.
    odom_velocity: Option<f64>,
    /// Already scaled to the 0-100 this message reports; `sensor_msgs/BatteryState` is 0-1.
    battery_percentage: Option<f64>,
    /// Engaged by a CHARGING `battery_state`, released by a DISCHARGING one ([`ChargingLatch`]
    /// has why). Clear until a sample arrives: with no battery report the robot is not known to
    /// be charging.
    charging: ChargingLatch,
    wifi: Option<WifiInfo>,
    /// Kept whole, not picked apart: `motor_status` IS a MotorStates, so the array and the instant
    /// it describes cannot be copied independently or get out of step.
    motor_states: Option<MotorStates>,
    /// Decoded in the callback rather than cached as a message, because the payload is two ints
    /// and validating on the way in means a short `data[]` never lands here.
    ///
    /// Fed by TWO topics: `policy_state` / `motion_state` from `mode`, `safety_state` from
    /// `safety_locked`. Each intake writes only its own fields, so neither topic can reset what the
    /// other reported.
    ///
    /// This carries NO indication of whether anything has ever been heard: 0 / 0 is both the
    /// initial value and a genuine "PPO / Stand". A receipt timestamp used to make the difference
    /// visible and was removed on request. If it comes back, the invariant that mattered was that
    /// a REJECTED sample must not refresh it, or a stream of malformed messages makes a frozen
    /// reading look live.
    low_level_mode: RobotLowLevelMode,
}

impl RobotStateData {
    pub fn new(
        config: Config,
        publisher: Publisher<RobotState>,
        clock: Clock,
        logger: Logger,
        mode_poll: ModePoll,
    ) -> Self {
        Self {
            config,
            publisher,
            clock,
            logger,
            mode_poll,
            tf: TfBuffer::default(),
            latch: LowBatteryLatch::default(),
            lifecycle: Lifecycle::default(),
            odom_velocity: None,
            battery_percentage: None,
            charging: ChargingLatch::default(),
            wifi: None,
            motor_states: None,
            low_level_mode: RobotLowLevelMode::default(),
        }
    }

    // --- sample intake, one per subscription -------------------------------------------------

    pub fn accept_odom(&mut self, forward_velocity: f64) {
        self.odom_velocity = Some(forward_velocity);
    }

    /// `percentage` is `sensor_msgs/BatteryState.percentage`, i.e. 0-1. It is scaled here, once,
    /// so nothing below has to remember which convention it is looking at.
    ///
    /// The scaling round-trips: syncai_driver_manager divides the BMS's 0-100 SoC by 100 to build
    /// the message, and this multiplies it back. Changing one side without the other gives a
    /// reading off by 100x.
    ///
    /// `charge` is its `power_supply_status`, which advances the [`ChargingLatch`]. Logged on a
    /// change of the latch only.
    pub fn accept_battery(&mut self, percentage: f32, charge: ChargeReading) {
        self.battery_percentage = Some(f64::from(percentage) * 100.0);
        if let Some(charging) = self.charging.update(charge) {
            log_info!(
                &self.logger,
                "[RobotStateNode] battery {} at {:.1}%",
                if charging {
                    "charging; state -> CHARGING"
                } else {
                    "discharging; no longer CHARGING"
                },
                f64::from(percentage) * 100.0,
            );
        }
    }

    pub fn accept_wifi(&mut self, info: WifiInfo) {
        self.wifi = Some(info);
    }

    pub fn accept_motor_states(&mut self, states: MotorStates) {
        self.motor_states = Some(states);
    }

    /// `data[0]` = policy state, `data[1]` = motion state. Anything shorter is REJECTED, not
    /// padded.
    ///
    /// Padding would be worse than dropping: 0 is a legitimate value on both indices (PPO /
    /// Stand), so a padded element is indistinguishable from a real reading and would let a
    /// malformed packet claim the robot is standing. There is no in-band way for a consumer to
    /// tell the difference.
    ///
    /// The real publisher cannot produce a short array — syncai_driver_manager demands two tokens
    /// and publishes nothing otherwise — so this guard is for a malformed or third-party publisher
    /// on the same topic. A rejected sample leaves the previous reading in place, and nothing
    /// downstream can tell that happened: `low_level_mode` carries no freshness field, so a
    /// malformed publisher talking over the real one is visible only in this warning.
    ///
    /// More than two elements is NOT an error: the MODE_STATE telemetry section runs until the
    /// next keyword, so a controller that starts reporting a third value widens this array
    /// additively. Read the two we understand, ignore the rest, warn about nothing.
    pub fn accept_low_level_mode(&mut self, data: &[i32]) {
        let [policy_state, motion_state, ..] = data else {
            log_warn!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[RobotStateNode] Ignoring `mode` sample with {} element(s); expected at least 2 \
                 (data[0] policy state, data[1] motion state). Holding the previous low_level_mode.",
                data.len(),
            );
            return;
        };

        // Field by field, not a fresh struct: `safety_state` comes from a different topic and a
        // `mode` sample must not reset it to false.
        self.low_level_mode.policy_state = *policy_state;
        self.low_level_mode.motion_state = *motion_state;
    }

    /// syncai_driver_manager's software safety lock, true = engaged. Ours, not the gait
    /// controller's — see RobotLowLevelMode.msg for why it rides in `low_level_mode` anyway.
    ///
    /// Logged on a change only: the source is latched and publishes only on a change, so every
    /// sample after the first is already one — but a late join or a driver restart re-delivers the
    /// current value, and that is not worth a line.
    pub fn accept_safety_locked(&mut self, engaged: bool) {
        if engaged != self.low_level_mode.safety_state {
            log_info!(
                &self.logger,
                "[RobotStateNode] safety lock {}",
                if engaged { "ENGAGED" } else { "released" },
            );
        }
        self.low_level_mode.safety_state = engaged;
    }

    pub fn accept_transform(&mut self, parent: &str, child: &str, transform: Transform) {
        if let Err(e) = self.tf.insert(parent, child, transform) {
            self.warn_invalid_transform(parent, child, e);
        }
    }

    fn warn_invalid_transform(&self, parent: &str, child: &str, reason: InvalidTransform) {
        log_warn!(
            self.logger.throttle(SAMPLE_LOG_THROTTLE),
            "[RobotStateNode] Ignoring TF '{parent}' -> '{child}': {reason}"
        );
    }

    // --- the state machine -------------------------------------------------------------------

    /// The localizer's latched facts: which map, whether its last guess converged, and the epoch
    /// that names its estimate.
    ///
    /// Nothing is decided here beyond storing it — a confirmation is withdrawn by comparison
    /// with the epoch it was given at, inside [`Lifecycle`]. What this adds is the log: the line
    /// that says WHY a confirmed robot is UNINITIALIZED again is the one an operator's "it was
    /// fine a minute ago" gets answered from.
    ///
    /// An identical sample is a replay (the topic is latched and re-delivered to a late joiner,
    /// and on every rematch after the localizer restarts) and is not worth a line.
    pub fn accept_localization_status(&mut self, sample: Localization) {
        if self.lifecycle.localization() == Some(&sample) {
            return;
        }
        let previous_epoch = self.lifecycle.localization().map(|l| l.epoch);
        let withdrawn = self.lifecycle.accept_localization(sample.clone());
        if withdrawn {
            log_warn!(
                &self.logger,
                "[RobotStateNode] position confirmation withdrawn: the localizer's estimate is \
                 no longer the one that was confirmed (pose_epoch {} -> {}, map '{}', converged \
                 {}); state -> UNINITIALIZED until an operator confirms again",
                previous_epoch.map_or_else(|| "none".to_owned(), |e| e.to_string()),
                sample.epoch,
                sample.map,
                sample.converged,
            );
            // The console must not show a confirmed robot for the rest of the publish period.
            self.publish();
        } else {
            log_info!(
                &self.logger,
                "[RobotStateNode] localization: map '{}', converged {}, pose_epoch {} \
                 (position confirmed: {})",
                sample.map,
                sample.converged,
                sample.epoch,
                self.lifecycle.is_confirmed(),
            );
        }
    }

    /// `set_robot_state`: the operator's switch. See [`Lifecycle::set_state`] for the rules.
    pub fn set_robot_state(&mut self, request: SetRobotState_Request) -> SetRobotState_Response {
        let target = match request.state {
            RobotStatus::IDLE => Target::Idle,
            RobotStatus::UNINITIALIZED => Target::Uninitialized,
            other => {
                log_warn!(
                    &self.logger,
                    "[RobotStateNode] set_robot_state refused: state {other} cannot be asked \
                     for (only IDLE = {} and UNINITIALIZED = {}; the rest are derived)",
                    RobotStatus::IDLE,
                    RobotStatus::UNINITIALIZED,
                );
                return SetRobotState_Response {
                    success: false,
                    code: SetRobotState_Response::INVALID_TARGET,
                    message: "Only a confirmed or an unconfirmed position can be asked for."
                        .to_owned(),
                    state: self.status_code(),
                };
            }
        };

        match self.lifecycle.set_state(target) {
            Ok(released) => {
                self.log_switch(target, &released);
                // Now, not on the next tick: the caller's very next read of robot_state should
                // show what it just did.
                self.publish();
                SetRobotState_Response {
                    success: true,
                    code: SetRobotState_Response::NONE,
                    message: switch_sentence(target, &released),
                    state: self.status_code(),
                }
            }
            Err(why) => {
                log_warn!(
                    &self.logger,
                    "[RobotStateNode] set_robot_state(IDLE) refused: {why:?} (localization: \
                     {:?}); the state stays {}",
                    self.lifecycle.localization(),
                    status_name(self.status()),
                );
                SetRobotState_Response {
                    success: false,
                    code: SetRobotState_Response::NOT_CONVERGED,
                    message: why.sentence().to_owned(),
                    state: self.status_code(),
                }
            }
        }
    }

    fn log_switch(&self, target: Target, released: &Released) {
        let epoch = self.lifecycle.localization().map(|l| l.epoch);
        match target {
            Target::Idle => log_info!(
                &self.logger,
                "[RobotStateNode] position confirmed by the operator at pose_epoch {:?}; state \
                 -> {}",
                epoch,
                status_name(self.status()),
            ),
            Target::Uninitialized => log_info!(
                &self.logger,
                "[RobotStateNode] position confirmation withdrawn by the operator; state -> {}",
                status_name(self.status()),
            ),
        }
        if let Some(fault) = &released.fault {
            log_info!(
                &self.logger,
                "[RobotStateNode] error released by the operator (task '{}' had failed: {})",
                fault.task_id,
                if fault.reason.is_empty() {
                    "no reason given"
                } else {
                    &fault.reason
                },
            );
        }
        if let Some(task) = &released.task {
            // Either a run whose end_task never came, or a switch under a live task that the
            // backend should have refused. From here the two look the same; say so.
            log_warn!(
                &self.logger,
                "[RobotStateNode] task '{task}' was still counted as running and has been \
                 dropped by the operator's switch; a later end_task for it will be ignored",
            );
        }
    }

    /// `begin_task`: may this task start? See [`Lifecycle::begin_task`].
    pub fn begin_task(&mut self, request: BeginTask_Request) -> BeginTask_Response {
        let pose_available = self.pose_available();
        let initialized = Rollup::initialized(self.lifecycle.is_confirmed(), pose_available);
        match self
            .lifecycle
            .begin_task(&request.task_id, request.needs_localization, initialized)
        {
            Ok(Begun::Started) => {
                log_info!(
                    &self.logger,
                    "[RobotStateNode] task '{}' accepted (needs localization: {}); state -> {}",
                    request.task_id,
                    request.needs_localization,
                    status_name(self.status()),
                );
                self.publish();
                BeginTask_Response {
                    accepted: true,
                    code: BeginTask_Response::NONE,
                    message: "The task was accepted.".to_owned(),
                }
            }
            Ok(Begun::AlreadyRunning) => {
                log_info!(
                    &self.logger,
                    "[RobotStateNode] task '{}' asked to begin again and is already running; \
                     accepted again (a retried call)",
                    request.task_id,
                );
                BeginTask_Response {
                    accepted: true,
                    code: BeginTask_Response::NONE,
                    message: "The task is already running.".to_owned(),
                }
            }
            Err(refusal) => {
                let (code, message) = match &refusal {
                    BeginRefusal::ErrorLatched(_) => (
                        BeginTask_Response::ERROR_LATCHED,
                        "The robot refused the task: an earlier task failed and the error has \
                         not been released.",
                    ),
                    BeginRefusal::NotInitialized => (
                        BeginTask_Response::NOT_INITIALIZED,
                        "The robot refused the task: its position on the map has not been \
                         confirmed by an operator.",
                    ),
                    BeginRefusal::Busy { .. } => (
                        BeginTask_Response::BUSY,
                        "The robot refused the task: another task is running.",
                    ),
                };
                log_warn!(
                    &self.logger,
                    "[RobotStateNode] task '{}' refused: {refusal:?} (needs localization: {}, \
                     position confirmed: {}, pose available: {pose_available}, localization: \
                     {:?}); the state stays {}",
                    request.task_id,
                    request.needs_localization,
                    self.lifecycle.is_confirmed(),
                    self.lifecycle.localization(),
                    status_name(self.status()),
                );
                BeginTask_Response {
                    accepted: false,
                    code,
                    message: message.to_owned(),
                }
            }
        }
    }

    /// `end_task`: how an accepted task ended. See [`Lifecycle::end_task`].
    pub fn end_task(&mut self, request: EndTask_Request) -> EndTask_Response {
        let outcome = match request.outcome {
            EndTask_Request::COMPLETED => Outcome::Completed,
            EndTask_Request::FAILED => Outcome::Failed,
            EndTask_Request::CANCELED => Outcome::Canceled,
            other => {
                // 0 is deliberately no outcome (EndTask.srv): a request nobody filled in must
                // not close a task as completed.
                log_warn!(
                    &self.logger,
                    "[RobotStateNode] end_task for '{}' refused: outcome {other} is not one of \
                     COMPLETED / FAILED / CANCELED; nothing changed",
                    request.task_id,
                );
                return EndTask_Response {
                    success: false,
                    message: "The report names no outcome.".to_owned(),
                };
            }
        };

        match self
            .lifecycle
            .end_task(&request.task_id, outcome, &request.reason)
        {
            Ended::Closed => {
                log_info!(
                    &self.logger,
                    "[RobotStateNode] task '{}' ended {outcome:?}; state -> {}",
                    request.task_id,
                    status_name(self.status()),
                );
                self.publish();
                EndTask_Response {
                    success: true,
                    message: "The task was closed.".to_owned(),
                }
            }
            Ended::Latched => {
                log_error!(
                    &self.logger,
                    "[RobotStateNode] task '{}' FAILED ({}); state -> ERROR, latched: every \
                     task is refused until an operator releases it with set_robot_state",
                    request.task_id,
                    if request.reason.trim().is_empty() {
                        "no reason given"
                    } else {
                        request.reason.trim()
                    },
                );
                self.publish();
                EndTask_Response {
                    success: true,
                    message: "The task was closed and the error is held.".to_owned(),
                }
            }
            Ended::NotRunning => {
                log_warn!(
                    &self.logger,
                    "[RobotStateNode] end_task({outcome:?}) for '{}' ignored: the robot is \
                     running {}. Nothing changed, and no error is latched from it",
                    request.task_id,
                    self.lifecycle
                        .task()
                        .map_or_else(|| "no task".to_owned(), |t| format!("task '{t}'")),
                );
                EndTask_Response {
                    success: false,
                    message: "The robot is not running that task.".to_owned(),
                }
            }
        }
    }

    /// There is a `map -> base_link` to report: the rollup's second condition for "initialized"
    /// ([`Rollup::initialized`]), asked here for the services, which run between ticks.
    fn pose_available(&self) -> bool {
        self.tf
            .robot_pose(
                &self.config.global_frame,
                &self.config.base_frame,
                &self.config.body_frame,
                &self.config.lidar_frame,
            )
            .is_ok()
    }

    fn rollup(&self, pose_available: bool) -> Status {
        health::derive_status(Rollup {
            error: self.lifecycle.fault().is_some(),
            initialized: Rollup::initialized(self.lifecycle.is_confirmed(), pose_available),
            running: self.lifecycle.task().is_some(),
            charging: self.charging.is_engaged(),
            low_battery: self.latch.is_engaged(),
        })
    }

    fn status(&self) -> Status {
        self.rollup(self.pose_available())
    }

    fn status_code(&self) -> u8 {
        status_code(self.status())
    }

    // --- the tick ------------------------------------------------------------------------------

    /// Latches first, then the build that reads them, so both describe the same tick. This is the
    /// only place the battery latch advances.
    ///
    /// Keeping the two apart gives any future dwell counter or rate-limited transition exactly one
    /// place it can live, ticking once per publish period rather than once per `build_state` call
    /// — which is also why the state machine's own transitions call [`Self::publish`] and not
    /// this.
    pub fn on_timer(&mut self) {
        self.update_health_latch();
        self.publish();
    }

    /// Build and send one `robot_state`. Called by the tick, and at once after every transition
    /// of the state machine: at the shipped 1 Hz a caller that confirmed the position, or a
    /// console that has just been told a task was refused, would otherwise read the old state
    /// for up to a second.
    fn publish(&self) {
        let msg = self.build_state();
        if let Err(e) = self.publisher.publish(msg) {
            log_error!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[RobotStateNode] publish robot_state failed: {e}"
            );
        }
    }

    pub fn poll_mode(&self) {
        self.mode_poll.tick();
    }

    fn update_health_latch(&mut self) {
        let change = self
            .latch
            .update(self.battery_percentage, self.config.battery_thresholds);

        match change {
            LatchChange::Held => {}
            LatchChange::NoUsableSample { sample: None } => log_warn!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[RobotStateNode] No battery_state received yet; holding low_battery latch at {}",
                self.latch.is_engaged(),
            ),
            LatchChange::NoUsableSample {
                sample: Some(percentage),
            } => log_warn!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[RobotStateNode] Unusable battery sample ({percentage}%); holding low_battery \
                 latch at {}",
                self.latch.is_engaged(),
            ),
            LatchChange::Engaged(percentage) => log_warn!(
                &self.logger,
                "[RobotStateNode] battery {percentage:.1}% below {:.1}%; state -> WARNING",
                self.config.battery_thresholds.warn(),
            ),
            LatchChange::Cleared(percentage) => log_info!(
                &self.logger,
                "[RobotStateNode] battery recovered to {percentage:.1}% (above {:.1}%)",
                self.config.battery_thresholds.clear(),
            ),
        }
    }

    /// Build the message from the current TF and cached samples. A pure read of the latch —
    /// advancing it is [`Self::update_health_latch`]'s job.
    fn build_state(&self) -> RobotState {
        let mut msg = RobotState {
            // Whole SECONDS, because this field is passed verbatim to GET /api/v1/robot/state. It
            // is a wall clock for the UI, not a sequence number: at the 10 Hz code default ten
            // consecutive messages carry the same value, so it cannot order samples or measure the
            // rate. Subscribe motor_states directly for sub-second resolution.
            timestamp: self.timestamp_seconds(),
            robot_id: self.config.robot_id.clone(),
            map: self.config.map.clone(),
            mode: self.mode_poll.reported_mode(),
            ..Default::default()
        };

        // A failed lookup used to abort the whole tick, which meant that before the localizer had
        // been relocalized nothing was published at all — no battery, no wifi, no joint
        // temperatures, precisely when an operator is trying to work out why the robot will not
        // localize. The message now goes out regardless, carrying an explicit "the pose is not
        // usable" marker instead.
        match self.tf.robot_pose(
            &self.config.global_frame,
            &self.config.base_frame,
            &self.config.body_frame,
            &self.config.lidar_frame,
        ) {
            Ok(ReportedPose {
                pose,
                planar_z_reason,
            }) => {
                msg.localization_valid = true;
                msg.localization_status.position = RobotPose {
                    x: pose.x,
                    y: pose.y,
                    z: pose.z,
                    yaw: pose.yaw,
                };
                // The planar z is 0 on every storey, which puts an upstairs robot on the ground
                // floor for every consumer. Worth a line, not a dropped pose.
                if let Some(e) = planar_z_reason {
                    log_warn!(
                        self.logger.throttle(TF_LOG_THROTTLE),
                        "[RobotStateNode] Height through the lidar unavailable ({e}); reporting \
                         the z of TF {} -> {}, which is planar",
                        self.config.global_frame,
                        self.config.base_frame,
                    );
                }
            }
            Err(e) => {
                // localization_status stays zero-initialised. Deliberately NOT the last known
                // pose: a stale pose with no age attached reads as a live one, whereas the map
                // origin is an obviously suspicious value.
                msg.localization_valid = false;
                log_warn!(
                    self.logger.throttle(TF_LOG_THROTTLE),
                    "[RobotStateNode] TF {} -> {} unavailable ({e}); publishing robot_state with \
                     localization_valid=false",
                    self.config.global_frame,
                    self.config.base_frame,
                );
            }
        }

        // The rollup of the state machine and the battery, most-severe-first
        // (health::derive_status has the precedence and why). `localization_valid` above is one
        // of its inputs and no longer its meaning: UNINITIALIZED is "no operator has confirmed
        // the position", which outlasts the TF by however long it takes someone to look.
        msg.state = status_code(self.rollup(msg.localization_valid));
        msg.localization_converged = self.lifecycle.is_converged();
        msg.state_message = self.lifecycle.message(msg.localization_valid);

        // Forward speed only (linear.x), not the speed magnitude: negative when reversing, and
        // lateral motion is ignored.
        msg.localization_status.velocity = self.odom_velocity.unwrap_or(0.0);
        msg.battery_status.battery_percentage = self.battery_percentage.unwrap_or(0.0);
        msg.network_status.wifi_info = wifi::wifi_info_json(self.wifi.as_ref());

        // Operator-facing detail; must not be forwarded into the REST payload — see RobotState.msg.
        // Left at an empty `states` and a 0 `timestamp` while syncai_driver_manager is down, which
        // is itself the useful signal.
        //
        // The one thing that is NOT verbatim is the unit: MotorStates.timestamp is nanoseconds on
        // the topic, and RobotState reports seconds everywhere else, so it is scaled here. Integer
        // division, deliberately — the field is a u64 and this message's other timestamp is whole
        // seconds too, so truncating is consistent rather than lossy in a new way. The sub-second
        // joint channel is the backend telemetry WebSocket, which reads the topic directly and
        // still gets nanoseconds.
        if let Some(motor_states) = &self.motor_states {
            msg.motor_status = motor_states.clone();
            msg.motor_status.timestamp = motor_states.timestamp / 1_000_000_000;
        }

        // Straight through, no branch: this defaults to 0 / 0 / false, which is what a consumer
        // sees before the first sample — and is indistinguishable from a genuine "PPO / Stand,
        // lock released", because the field carries no freshness information.
        //
        // Nothing here judges freshness, and nothing here touches msg.state: RobotStatus has no
        // STALE value, UNINITIALIZED belongs to localization, and WARNING carries no reason field,
        // so a third cause folded into it would be unreadable. Same split as localization_valid vs
        // state, where the precise field answers the precise question and state stays the rollup.
        msg.low_level_mode = self.low_level_mode.clone();

        msg
    }

    fn timestamp_seconds(&self) -> u64 {
        u64::try_from(self.clock.now().nsec.div_euclid(1_000_000_000)).unwrap_or(0)
    }
}

fn status_code(status: Status) -> u8 {
    match status {
        Status::Error => RobotStatus::ERROR,
        Status::Uninitialized => RobotStatus::UNINITIALIZED,
        Status::Running => RobotStatus::RUNNING,
        Status::Charging => RobotStatus::CHARGING,
        Status::Warning => RobotStatus::WARNING,
        Status::Idle => RobotStatus::IDLE,
    }
}

/// The constant's name, for the log: a line that says "state -> 4" sends its reader to the .msg.
fn status_name(status: Status) -> &'static str {
    match status {
        Status::Error => "ERROR",
        Status::Uninitialized => "UNINITIALIZED",
        Status::Running => "RUNNING",
        Status::Charging => "CHARGING",
        Status::Warning => "WARNING",
        Status::Idle => "IDLE",
    }
}

/// What the operator is told their switch did. Shown verbatim by the console.
fn switch_sentence(target: Target, released: &Released) -> String {
    let mut sentence = match target {
        Target::Idle => "The robot's position is confirmed.",
        Target::Uninitialized => "The robot's position is no longer confirmed.",
    }
    .to_owned();
    if released.fault.is_some() {
        sentence.push_str(" The error was released.");
    }
    sentence
}
