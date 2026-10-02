use rclrs::*;
use std::sync::Arc;
use std::time::Duration;

use super::health::{self, Thresholds};

/// The range a rate parameter may turn into.
///
/// Both ends are guards the C++ version does not need and does not have: `duration_cast` saturates
/// where `Duration::from_secs_f64` panics, and an rclcpp wall timer tolerates a zero period.
/// `1 / f64::MIN_POSITIVE` is ~4.5e307 seconds — finite, so it passes an `is_finite` check, and far
/// past what a `Duration` can hold.
const MIN_PERIOD: Duration = Duration::from_millis(1);
const MAX_PERIOD: Duration = Duration::from_secs(3600);

/// Hz -> timer period, with the C++ version's clamp.
///
/// A non-positive rate would make the timer either fail or fire as fast as the executor allows, so
/// it is clamped to a slow but harmless 1 Hz rather than trusted. NaN and infinity take the same
/// path, and the resulting period is then held inside [`MIN_PERIOD`]..=[`MAX_PERIOD`].
pub fn period_from_rate(rate_hz: f64) -> Duration {
    let safe_rate = if rate_hz > 0.0 && rate_hz.is_finite() {
        rate_hz
    } else {
        1.0
    };

    let seconds = 1.0 / safe_rate;
    if !seconds.is_finite() || seconds <= 0.0 {
        return Duration::from_secs(1);
    }

    // Clamped BEFORE the conversion: from_secs_f64 panics on a value that overflows a Duration,
    // so clamping the Duration afterwards would be too late.
    Duration::from_secs_f64(seconds.clamp(MIN_PERIOD.as_secs_f64(), MAX_PERIOD.as_secs_f64()))
}

/// The effective values the node runs on, after clamping and validation.
///
/// Held as plain owned values rather than read through the parameter handles on every tick: they
/// cannot change (see [`Parameters`]), and the battery thresholds may differ from what was
/// declared when the supplied pair was rejected.
pub struct Config {
    pub robot_id: String,
    /// Stamped into `RobotState.map`. Despite the name the launch file usually puts a *path* here
    /// (the map YAML from the INI's `[map] map`), which goes through to the REST payload verbatim.
    pub map: String,
    /// TF frame names are not namespaced by ROS; the launch file prefixes `base_frame` with
    /// `<robot_id>/` and leaves `global_frame` as plain `map`.
    pub global_frame: String,
    pub base_frame: String,
    pub transform_tolerance: f64,
    pub publish_period: Duration,
    pub mode_poll_period: Duration,
    pub battery_thresholds: Thresholds,
}

/// The node's parameters, all of them load-time only.
///
/// **Difference from the C++ version:** these are declared `read_only()`. The C++ node reads every
/// parameter once in `initParameters()` and has no `add_on_set_parameters_callback`, so
/// `ros2 param set` there appears to succeed and silently changes nothing. Declaring them
/// read-only makes that same fact visible: the set is rejected instead of ignored. Values still
/// come from the params file and the launch file's overrides, which are applied at declaration
/// time, before the parameter is locked.
///
/// The handles are kept for the node's lifetime because a parameter is undeclared when its handle
/// drops.
pub struct Parameters {
    _robot_id: ReadOnlyParameter<Arc<str>>,
    _map: ReadOnlyParameter<Arc<str>>,
    _global_frame: ReadOnlyParameter<Arc<str>>,
    _base_frame: ReadOnlyParameter<Arc<str>>,
    _transform_tolerance: ReadOnlyParameter<f64>,
    _publish_rate: ReadOnlyParameter<f64>,
    _mode_poll_rate: ReadOnlyParameter<f64>,
    _low_battery_warn_percentage: ReadOnlyParameter<f64>,
    _low_battery_clear_percentage: ReadOnlyParameter<f64>,
}

impl Parameters {
    pub fn declare(node: &Node) -> Result<(Self, Config), DeclarationError> {
        let text = |name: &str, default: &str, description: &str| {
            node.declare_parameter(name)
                // Turbofished: nothing downstream pins the element type, so a bare
                // Arc::from leaves the parameter as ReadOnlyParameter<Arc<_>>
                .default(Arc::<str>::from(default))
                .description(description)
                .read_only()
        };
        let number = |name: &str, default: f64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .description(description)
                .read_only()
        };

        let robot_id = text("robot_id", "", "stamped into RobotState.robot_id")?;
        let map = text(
            "map",
            "",
            "stamped into RobotState.map; usually a map YAML path",
        )?;
        let global_frame = text("global_frame", "map", "TF frame the pose is reported in")?;
        let base_frame = text(
            "base_frame",
            "base_link",
            "TF frame of the robot; the launch file prefixes it with <robot_id>/",
        )?;
        let transform_tolerance = number(
            "transform_tolerance",
            0.1,
            "TF lookup tolerance, in seconds",
        )?;
        // 10.0 is the C++ code default, and the shipped params file overrides it with 1.0 — so a
        // launched node runs at 1 Hz despite this number and despite every "10 Hz" in the docs.
        let publish_rate = number("publish_rate", 10.0, "robot_state publish rate, in Hz")?;
        // Deliberately decoupled from publish_rate rather than tied to it: every get_mode call
        // makes sys_manager spawn `byobu has-session` subprocesses, and the mode only changes over
        // a tens-of-seconds switch. Raising publish_rate must not drag this up with it.
        let mode_poll_rate = number("mode_poll_rate", 1.0, "get_mode poll rate, in Hz")?;
        let low_battery_warn_percentage = number(
            "low_battery_warn_percentage",
            health::DEFAULT_WARN_PERCENTAGE,
            "latch state=WARNING below this battery percentage (0-100)",
        )?;
        let low_battery_clear_percentage = number(
            "low_battery_clear_percentage",
            health::DEFAULT_CLEAR_PERCENTAGE,
            "release the WARNING latch above this battery percentage (0-100)",
        )?;

        let (battery_thresholds, rejected) = health::validate(
            low_battery_warn_percentage.get(),
            low_battery_clear_percentage.get(),
        );
        if rejected {
            log_error!(
                node.logger(),
                "[Parameters] low_battery_clear_percentage ({:.1}) must exceed \
                 low_battery_warn_percentage ({:.1}); falling back to {:.1}/{:.1}",
                low_battery_clear_percentage.get(),
                low_battery_warn_percentage.get(),
                battery_thresholds.warn,
                battery_thresholds.clear,
            );
        }

        let config = Config {
            robot_id: robot_id.get().to_string(),
            map: map.get().to_string(),
            global_frame: global_frame.get().to_string(),
            base_frame: base_frame.get().to_string(),
            transform_tolerance: transform_tolerance.get(),
            publish_period: period_from_rate(publish_rate.get()),
            mode_poll_period: period_from_rate(mode_poll_rate.get()),
            battery_thresholds,
        };

        log_info!(
            node.logger(),
            "[Parameters] robot_id: '{}', map: '{}', TF {} -> {} (tolerance {:.2} s)",
            config.robot_id,
            config.map,
            config.global_frame,
            config.base_frame,
            config.transform_tolerance,
        );
        log_info!(
            node.logger(),
            "[Parameters] publish every {:?}, get_mode poll every {:?}, \
             low battery: warn below {:.1}%, clear above {:.1}%",
            config.publish_period,
            config.mode_poll_period,
            config.battery_thresholds.warn,
            config.battery_thresholds.clear,
        );

        Ok((
            Self {
                _robot_id: robot_id,
                _map: map,
                _global_frame: global_frame,
                _base_frame: base_frame,
                _transform_tolerance: transform_tolerance,
                _publish_rate: publish_rate,
                _mode_poll_rate: mode_poll_rate,
                _low_battery_warn_percentage: low_battery_warn_percentage,
                _low_battery_clear_percentage: low_battery_clear_percentage,
            },
            config,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rate_becomes_its_period() {
        assert_eq!(period_from_rate(1.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(10.0), Duration::from_millis(100));
        assert_eq!(period_from_rate(2.0), Duration::from_millis(500));
    }

    #[test]
    fn a_non_positive_rate_is_clamped_to_one_hz_rather_than_trusted() {
        assert_eq!(period_from_rate(0.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(-5.0), Duration::from_secs(1));
    }

    /// `Duration::from_secs_f64` panics on NaN and on the infinity a denormal rate divides to, so
    /// a hostile params file must not reach it.
    #[test]
    fn unusable_rates_do_not_panic() {
        assert_eq!(period_from_rate(f64::NAN), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::INFINITY), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::NEG_INFINITY), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::MIN_POSITIVE), MAX_PERIOD);
        // A rate so high the period rounds to zero would give a timer that never stops firing
        assert_eq!(period_from_rate(1e9), MIN_PERIOD);
    }
}
