//! The low-battery latch — the node's only derived state beyond the latest sample of each input.
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

#[cfg(test)]
mod tests {
    use super::*;

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
