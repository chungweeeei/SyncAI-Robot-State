//! The two battery latches — low battery and charging — and the `state` rollup that reads them
//! together with the state machine's facts (`lifecycle.rs`: confirmed position, running task,
//! latched failure).
//!
//! Pure: no ROS types, no clock, no logging. The caller decides what to say about the
//! [`LatchChange`] it gets back.

use std::fmt;

/// The hysteresis band, in percent (0-100).
///
/// A bare comparison would flip `state` between IDLE and WARNING on every publish for a pack
/// sitting on the threshold — once a second at the shipped 1 Hz, ten times a second at the 10 Hz
/// code default — and a state that flaps is one nobody can act on.
///
/// The fields are private so that `clear > warn` holds for every value that exists: the only ways
/// to get one are [`Thresholds::new`], which checks, and [`Thresholds::default`], which is known
/// good. The latch can therefore rely on the band being a band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Thresholds {
    warn: f64,
    clear: f64,
}

/// The default pair, and the fallback when a params file supplies an unusable one.
///
/// 20% is not a new number: it is the threshold in syncai_driver_manager's unwired
/// "soc < 20%" safety TODO, the one the reference GaitMPC bridge acts on, and the one the
/// frontend status strip hardcodes for its battery colour.
pub const DEFAULT_WARN_PERCENTAGE: f64 = 20.0;
pub const DEFAULT_CLEAR_PERCENTAGE: f64 = 25.0;

impl Default for Thresholds {
    fn default() -> Self {
        Self {
            warn: DEFAULT_WARN_PERCENTAGE,
            clear: DEFAULT_CLEAR_PERCENTAGE,
        }
    }
}

impl Thresholds {
    /// Build a band, rejecting one that is not.
    ///
    /// Equal thresholds mean no hysteresis at all (the state flaps on sensor noise at the publish
    /// rate); a clear value below the warn value means the latch can never clear. Neither is worth
    /// honouring silently, so both come back as an [`InvalidBand`] for the caller to log and fall
    /// back from, as the C++ version does at startup.
    pub fn new(warn: f64, clear: f64) -> Result<Self, InvalidBand> {
        if clear > warn {
            Ok(Self { warn, clear })
        } else {
            Err(InvalidBand { warn, clear })
        }
    }

    /// Latch WARNING below this.
    pub fn warn(self) -> f64 {
        self.warn
    }

    /// Release the latch only above this.
    pub fn clear(self) -> f64 {
        self.clear
    }
}

/// A warn/clear pair that is not a hysteresis band. Carries the pair so the log line can show
/// what was supplied next to what is used instead.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidBand {
    pub warn: f64,
    pub clear: f64,
}

impl fmt::Display for InvalidBand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "low_battery_clear_percentage ({:.1}) must exceed low_battery_warn_percentage ({:.1})",
            self.clear, self.warn
        )
    }
}

/// What one tick did to the latch. Everything but [`Self::Held`] is worth a log line.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LatchChange {
    /// Either inside the hysteresis band, or already on the side the reading calls for.
    Held,
    /// No evidence either way, so the latch keeps whatever it had. Carries what was seen, because
    /// "no `battery_state` has ever arrived" and "a sample arrived and reads 0" are worth telling
    /// apart in the log.
    NoUsableSample {
        /// `None` when no `battery_state` has ever arrived; otherwise the reading that was
        /// rejected (always 0 or below, see [`LowBatteryLatch::update`]).
        sample: Option<f64>,
    },
    /// Crossed below `warn`; carries the reading that did it.
    Engaged(f64),
    /// Recovered above `clear`; carries the reading that did it.
    Cleared(f64),
}

/// Latched "the battery is low", driving `state = WARNING`.
///
/// This node only REPORTS. Crossing the threshold does not lie the robot down or block cmd_vel —
/// syncai_driver_manager's `triggerSafeShutdown()` still has zero call sites, and who owns that
/// actuation is deliberately still open.
#[derive(Default)]
pub struct LowBatteryLatch {
    engaged: bool,
}

impl LowBatteryLatch {
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Advance the latch from the latest battery reading, in percent (0-100).
    ///
    /// Two ways to read a low battery that is not one, both of which would latch WARNING on a
    /// healthy robot, so neither moves the latch:
    ///
    /// 1. **No sample at all.** `battery_percentage` is reported as 0.0 until the first
    ///    `battery_state` arrives, so a robot whose driver_manager has simply not started would
    ///    otherwise look completely flat.
    /// 2. **`percentage == 0`.** syncai_driver_manager parses the BMS section with a bare `strtod`
    ///    rather than the validating parser it uses for every other section, so a non-numeric or
    ///    empty token silently publishes 0.0. A robot at a genuine 0% is not powered on to be
    ///    asked about.
    pub fn update(&mut self, sample: Option<f64>, thresholds: Thresholds) -> LatchChange {
        let Some(percentage) = sample.filter(|p| *p > 0.0) else {
            return LatchChange::NoUsableSample { sample };
        };

        if !self.engaged && percentage < thresholds.warn {
            self.engaged = true;
            LatchChange::Engaged(percentage)
        } else if self.engaged && percentage > thresholds.clear {
            self.engaged = false;
            LatchChange::Cleared(percentage)
        } else {
            // Between the two thresholds the latch is intentionally left alone — that gap is the
            // hysteresis band.
            LatchChange::Held
        }
    }
}

/// What one `battery_state` says about charging: its `power_supply_status`, reduced to the two
/// values the [`ChargingLatch`] acts on. Kept free of the message's constants, as [`Status`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChargeReading {
    /// `POWER_SUPPLY_STATUS_CHARGING`: the pack's net current over +0.1 A.
    Charging,
    /// `POWER_SUPPLY_STATUS_DISCHARGING`: the net current under -0.1 A.
    Discharging,
    /// Anything else, chiefly `NOT_CHARGING`: the current inside the deadband.
    Neither,
}

/// Latched "on the charger" (2026-11-18), driving `state = CHARGING`.
///
/// syncai_driver_manager derives `power_supply_status` from the sign of the pack's NET current
/// with a 0.1 A deadband (`ChargeState::from_current`). A nearly full pack's charge current
/// tapers into that deadband with the contacts still made, so a bare reading would drop a robot
/// that is sitting on its charger from `CHARGING` to `IDLE` as it fills.
///
/// So it engages on a `Charging` reading and releases only on a `Discharging` one: a robot taken
/// off the charger draws its own load and reads `Discharging` within a sample. Not a percentage
/// rule (`charge.xml`'s WaitForCharging accepts "not discharging above 95 %" because it knows the
/// robot has just lain down on the charger; this node does not, and a full robot standing idle
/// elsewhere would read `CHARGING` by it). The price: a robot whose current sits in the deadband
/// for some other reason after a charge keeps reading `CHARGING` until it next discharges.
#[derive(Default)]
pub struct ChargingLatch {
    engaged: bool,
}

impl ChargingLatch {
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Advance the latch; `Some(new value)` when it changed.
    pub fn update(&mut self, reading: ChargeReading) -> Option<bool> {
        let next = match reading {
            ChargeReading::Charging => true,
            ChargeReading::Discharging => false,
            ChargeReading::Neither => self.engaged,
        };
        if next == self.engaged {
            None
        } else {
            self.engaged = next;
            Some(next)
        }
    }
}

/// The coarse `state` rollup, one variant per `RobotStatus` value this node emits. Kept free of
/// the message's constants so the precedence can be tested without ROS; `state.rs` maps it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Error,
    Uninitialized,
    Running,
    Charging,
    Warning,
    Idle,
}

/// Everything the rollup reads, named: five bools in a row at a call site are five chances to
/// pass them in the wrong order, and every wrong order type-checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rollup {
    /// A failure is latched (`Lifecycle::fault`).
    pub error: bool,
    /// An operator has confirmed the position AND there is a pose to report. See
    /// [`Rollup::initialized`].
    pub initialized: bool,
    /// A task is running (`Lifecycle::task`).
    pub running: bool,
    /// The [`ChargingLatch`]: engaged by the charger delivering more than the robot draws, held
    /// through the taper of a filling pack.
    pub charging: bool,
    /// The [`LowBatteryLatch`].
    pub low_battery: bool,
}

impl Rollup {
    /// The one definition of "initialized", shared by the rollup and by `begin_task`'s gate so
    /// that the published state and the answer a task gets cannot disagree.
    ///
    /// `confirmed` is the operator's confirmation of the localizer's current estimate.
    /// `pose_available` is the old meaning of UNINITIALIZED, kept as a second condition: with no
    /// `map -> base_link` the pose in the message is a zero placeholder, and a state that claimed
    /// otherwise would be lying about the very field beside it. In practice it cannot hold a
    /// confirmed robot back — a converged guess needs the odometry the transform comes with —
    /// which is why it can be a condition here without being a reason given to the operator.
    pub fn initialized(confirmed: bool, pose_available: bool) -> bool {
        confirmed && pose_available
    }
}

/// Most-severe-first.
///
/// `Error` outranks everything (2026-11): it is the one state that stops EVERY task and that
/// only a person can end, so nothing may sit in front of it — an operator who sees
/// UNINITIALIZED confirms the position and still has a robot that refuses every task.
///
/// `Uninitialized` outranks the rest: "nobody has confirmed where the robot is" is the more
/// fundamental fact, and above `Running` on purpose. A consumer can then read "neither `Error`
/// nor `Uninitialized`" as "the position is confirmed" without a field for it. The price is that
/// a posture task run on an unconfirmed robot reads `Uninitialized` while it runs; `state_message`
/// and the console's own job bar say that it does.
///
/// `Running` outranks the battery states: what the robot is doing matters more to someone
/// deciding whether to send it a task than how full it is, and the battery percentage is in the
/// message either way.
///
/// `Charging` outranks `Warning` (2026-11-18): a low pack on its charger is the problem being
/// solved, not one to raise. The low-battery latch keeps running underneath, so a robot taken
/// off the charger still low reads `Warning` again on the next tick.
pub fn derive_status(facts: Rollup) -> Status {
    if facts.error {
        Status::Error
    } else if !facts.initialized {
        Status::Uninitialized
    } else if facts.running {
        Status::Running
    } else if facts.charging {
        Status::Charging
    } else if facts.low_battery {
        Status::Warning
    } else {
        Status::Idle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_charging_latch_engages_only_on_a_charge_current() {
        let mut latch = ChargingLatch::default();
        // A full robot standing idle, its current in the deadband, has not been seen charging
        assert_eq!(latch.update(ChargeReading::Neither), None);
        assert!(!latch.is_engaged());
        assert_eq!(latch.update(ChargeReading::Charging), Some(true));
        assert!(latch.is_engaged());
        assert_eq!(latch.update(ChargeReading::Charging), None);
    }

    #[test]
    fn a_filling_pack_on_the_charger_stays_charging() {
        let mut latch = ChargingLatch::default();
        latch.update(ChargeReading::Charging);
        // The charge current tapers into the deadband as the pack fills
        assert_eq!(latch.update(ChargeReading::Neither), None);
        assert!(latch.is_engaged());
    }

    #[test]
    fn taken_off_the_charger_it_releases_on_the_first_discharge() {
        let mut latch = ChargingLatch::default();
        latch.update(ChargeReading::Charging);
        latch.update(ChargeReading::Neither);
        assert_eq!(latch.update(ChargeReading::Discharging), Some(false));
        assert!(!latch.is_engaged());
        assert_eq!(latch.update(ChargeReading::Neither), None);
        assert!(!latch.is_engaged());
    }

    /// A confirmed robot at rest on a healthy pack; each test changes what it is about.
    const AT_REST: Rollup = Rollup {
        error: false,
        initialized: true,
        running: false,
        charging: false,
        low_battery: false,
    };

    #[test]
    fn a_latched_error_outranks_everything() {
        for initialized in [false, true] {
            for running in [false, true] {
                let facts = Rollup {
                    error: true,
                    initialized,
                    running,
                    charging: true,
                    low_battery: true,
                };
                assert_eq!(derive_status(facts), Status::Error);
            }
        }
    }

    #[test]
    fn an_unconfirmed_robot_is_uninitialized_whatever_else_is_true() {
        for running in [false, true] {
            for charging in [false, true] {
                for low_battery in [false, true] {
                    let facts = Rollup {
                        initialized: false,
                        running,
                        charging,
                        low_battery,
                        ..AT_REST
                    };
                    assert_eq!(derive_status(facts), Status::Uninitialized);
                }
            }
        }
    }

    #[test]
    fn a_running_task_outranks_the_battery() {
        let facts = Rollup {
            running: true,
            charging: true,
            low_battery: true,
            ..AT_REST
        };
        assert_eq!(derive_status(facts), Status::Running);
    }

    #[test]
    fn charging_outranks_a_low_battery() {
        let charging = Rollup {
            charging: true,
            ..AT_REST
        };
        assert_eq!(derive_status(charging), Status::Charging);
        assert_eq!(
            derive_status(Rollup {
                low_battery: true,
                ..charging
            }),
            Status::Charging
        );
    }

    #[test]
    fn a_low_battery_off_the_charger_warns() {
        assert_eq!(
            derive_status(Rollup {
                low_battery: true,
                ..AT_REST
            }),
            Status::Warning
        );
        assert_eq!(derive_status(AT_REST), Status::Idle);
    }

    #[test]
    fn initialized_needs_both_the_confirmation_and_a_pose() {
        assert!(Rollup::initialized(true, true));
        assert!(!Rollup::initialized(true, false));
        assert!(!Rollup::initialized(false, true));
        assert!(!Rollup::initialized(false, false));
    }

    #[test]
    fn a_healthy_pack_leaves_the_latch_clear() {
        let mut latch = LowBatteryLatch::default();
        assert_eq!(
            latch.update(Some(80.0), Thresholds::default()),
            LatchChange::Held
        );
        assert!(!latch.is_engaged());
    }

    #[test]
    fn the_band_between_the_thresholds_holds_whatever_the_latch_had() {
        let mut latch = LowBatteryLatch::default();
        let thresholds = Thresholds::default();

        // Climbing into the band from above never latches
        assert_eq!(latch.update(Some(22.0), thresholds), LatchChange::Held);
        assert!(!latch.is_engaged());

        // Latch below 20, then 22 is inside the band again and must NOT clear it
        assert_eq!(
            latch.update(Some(19.0), thresholds),
            LatchChange::Engaged(19.0)
        );
        assert!(latch.is_engaged());
        assert_eq!(latch.update(Some(22.0), thresholds), LatchChange::Held);
        assert!(latch.is_engaged());

        // Only above 25 does it clear
        assert_eq!(
            latch.update(Some(26.0), thresholds),
            LatchChange::Cleared(26.0)
        );
        assert!(!latch.is_engaged());
    }

    #[test]
    fn a_missing_sample_is_not_a_flat_battery() {
        let mut latch = LowBatteryLatch::default();
        assert_eq!(
            latch.update(None, Thresholds::default()),
            LatchChange::NoUsableSample { sample: None }
        );
        assert!(!latch.is_engaged());
    }

    /// A corrupt BMS token publishes 0.0, which must not be read as an empty pack — and must not
    /// clear an already-latched warning either.
    #[test]
    fn a_zero_reading_moves_the_latch_in_neither_direction() {
        let mut latch = LowBatteryLatch::default();
        let thresholds = Thresholds::default();

        assert_eq!(
            latch.update(Some(0.0), thresholds),
            LatchChange::NoUsableSample { sample: Some(0.0) }
        );
        assert!(!latch.is_engaged());

        latch.update(Some(19.0), thresholds);
        assert!(latch.is_engaged());
        latch.update(Some(0.0), thresholds);
        assert!(latch.is_engaged());
    }

    #[test]
    fn a_band_is_accepted_as_supplied() {
        let thresholds = Thresholds::new(30.0, 40.0).unwrap();
        assert_eq!(thresholds.warn(), 30.0);
        assert_eq!(thresholds.clear(), 40.0);
    }

    /// Equal or inverted pairs are rejected as a pair, carrying what was supplied for the log.
    #[test]
    fn an_unusable_band_is_rejected() {
        assert_eq!(
            Thresholds::new(30.0, 30.0),
            Err(InvalidBand {
                warn: 30.0,
                clear: 30.0,
            })
        );
        assert_eq!(
            Thresholds::new(30.0, 20.0),
            Err(InvalidBand {
                warn: 30.0,
                clear: 20.0,
            })
        );
    }

    #[test]
    fn the_defaults_are_themselves_a_valid_band() {
        assert_eq!(
            Thresholds::new(DEFAULT_WARN_PERCENTAGE, DEFAULT_CLEAR_PERCENTAGE),
            Ok(Thresholds::default())
        );
    }

    #[test]
    fn an_invalid_band_names_both_values() {
        let text = InvalidBand {
            warn: 30.0,
            clear: 20.0,
        }
        .to_string();
        assert!(text.contains("20.0") && text.contains("30.0"), "{text}");
    }
}
