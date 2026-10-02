//! The worker payload and the build-and-publish tick.
//!
//! Everything here runs on the node's single [`Worker`], so the five sample caches, the TF buffer
//! and the health latch need no locking at all — the worker is rclrs's equivalent of an rclcpp
//! `MutuallyExclusive` callback group and guarantees its callbacks never overlap. That replaces
//! the C++ version's `mutex_`; see the threading note in `mod.rs` for why one worker is enough
//! here when the C++ needed two callback groups.

use rclrs::*;
use ros_env::syncai_common::msg::{
    MotorStates, RobotLowLevelMode, RobotPose, RobotState, RobotStatus,
};
use std::time::Duration;

use super::health::{LatchChange, LowBatteryLatch};
use super::mode_poll::ModePoll;
use super::parameters::Config;
use super::tf::{InvalidTransform, TfBuffer, Transform};
use super::wifi::{self, WifiInfo};

// The C++ throttles these at 2 s (TF) and 5 s (everything else); matched so the log reads the same
const TF_LOG_THROTTLE: Duration = Duration::from_secs(2);
const SAMPLE_LOG_THROTTLE: Duration = Duration::from_secs(5);

/// Everything the node holds: the latest sample of each input, the TF tree, one latch, and the
/// handles needed to publish.
///
/// The node "holds no state beyond the latest sample" apart from [`LowBatteryLatch`], and keeping
/// that true is why `low_level_mode` is carried through untouched — two integers, no verdict, no
/// freshness field.
pub struct RobotStateData {
    config: Config,
    publisher: Publisher<RobotState>,
    clock: Clock,
    logger: Logger,
    mode_poll: ModePoll,

    tf: TfBuffer,
    latch: LowBatteryLatch,

    /// Forward speed from `odom`. The whole Odometry message is not kept: only `linear.x` is ever
    /// read, so there is nothing else to hold alive.
    odom_velocity: Option<f64>,
    /// Already scaled to the 0-100 this message reports; `sensor_msgs/BatteryState` is 0-1.
    battery_percentage: Option<f64>,
    wifi: Option<WifiInfo>,
    /// Kept whole, not picked apart: `motor_status` IS a MotorStates, so the array and the instant
    /// it describes cannot be copied independently or get out of step.
    motor_states: Option<MotorStates>,
    /// Decoded in the callback rather than cached as a message, because the payload is two ints
    /// and validating on the way in means a short `data[]` never lands here.
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
            odom_velocity: None,
            battery_percentage: None,
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
    pub fn accept_battery(&mut self, percentage: f32) {
        self.battery_percentage = Some(f64::from(percentage) * 100.0);
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

        self.low_level_mode = RobotLowLevelMode {
            policy_state: *policy_state,
            motion_state: *motion_state,
        };
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

    // --- the tick ------------------------------------------------------------------------------

    /// Latches first, then the build that reads them, so both describe the same tick. This is the
    /// only place the latch advances.
    ///
    /// Keeping the two apart gives any future dwell counter or rate-limited transition exactly one
    /// place it can live, ticking once per publish rather than once per `build_state` call.
    pub fn on_timer(&mut self) {
        self.update_health_latch();
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
            LatchChange::NoUsableSample {
                have_sample,
                percentage,
            } => log_warn!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[RobotStateNode] No usable battery sample (have_sample={have_sample}, \
                 percentage={percentage}); holding low_battery latch at {}",
                self.latch.is_engaged(),
            ),
            LatchChange::Engaged(percentage) => log_warn!(
                &self.logger,
                "[RobotStateNode] battery {percentage:.1}% below {:.1}%; state -> WARNING",
                self.config.battery_thresholds.warn,
            ),
            LatchChange::Cleared(percentage) => log_info!(
                &self.logger,
                "[RobotStateNode] battery recovered to {percentage:.1}% (above {:.1}%)",
                self.config.battery_thresholds.clear,
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
        match self
            .tf
            .pose_of(&self.config.global_frame, &self.config.base_frame)
        {
            Ok(pose) => {
                msg.localization_valid = true;
                msg.localization_status.position = RobotPose {
                    x: pose.x,
                    y: pose.y,
                    z: pose.z,
                    yaw: pose.yaw,
                };
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

        // State derivation, most-severe-first. UNINITIALIZED wins over WARNING because "we do not
        // know where the robot is" is the more fundamental fact — a low battery is worth
        // reporting, but not at the cost of hiding that the pose in this very message is a zero
        // placeholder. Consumers that need the precise answer read localization_valid; state is
        // the coarse rollup of it.
        //
        // TODO: RUNNING and ERROR are not derived yet. CHARGING cannot be: syncai_driver_manager
        //       hardcodes BatteryState.power_supply_status to UNKNOWN, and the only other
        //       candidate is the sign of `current`, whose convention is undocumented in both this
        //       port and the reference implementation.
        msg.state = if !msg.localization_valid {
            RobotStatus::UNINITIALIZED
        } else if self.latch.is_engaged() {
            RobotStatus::WARNING
        } else {
            RobotStatus::IDLE
        };

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

        // Straight through, no branch: this defaults to 0 / 0, which is what a consumer sees
        // before the first sample — and is indistinguishable from a genuine "PPO / Stand", because
        // the field carries no freshness information.
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
