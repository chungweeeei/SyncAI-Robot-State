//! The robot's state machine: whether an operator has confirmed the position, whether a task is
//! running, whether a failure is latched — and so whether a task may start.
//!
//! Pure: no ROS types, no clock, no logging. The services in `state.rs` call in, log what came
//! back and publish; [`super::health::derive_status`] rolls the facts held here up with the
//! battery into the one `state` the message carries.
//!
//! # Why this node holds it
//!
//! Until 2026-11 `state` was derived from the TF tree and the battery, and UNINITIALIZED meant
//! only "`map -> base_link` has never been seen". The localizer broadcasts that transform from
//! Point-LIO's first odometry on — the identity before anything has converged, the previous map's
//! correction after a map switch — so the state read IDLE a few seconds after boot and stayed
//! IDLE through a switch, and a task (a scheduled one above all, which no operator is watching)
//! drove off from a pose nobody had looked at.
//!
//! The machine is here rather than in the localizer because most of it is not about
//! localization: a task never touches the localizer, and RUNNING / ERROR would have nothing to
//! do there. The localizer reports facts (`localization_status`: which map, has the last guess
//! converged, an epoch naming the estimate) and this node decides.
//!
//! ```text
//!   start ──► UNINITIALIZED ──set_state(Idle), needs converged──► IDLE
//!   IDLE ──begin_task accepted──► RUNNING ──end_task(Completed / Canceled)──► IDLE
//!   RUNNING ──end_task(Failed)──► ERROR ──set_state──► IDLE or UNINITIALIZED
//!   IDLE ──another pose_epoch / set_state(Uninitialized)──► UNINITIALIZED
//! ```
//!
//! # In memory only
//!
//! Nothing here is written to disk, so a restart of this node is UNINITIALIZED with no error and
//! no task. That is the wanted behaviour for the restarts that happen — a boot, a mode switch,
//! `restart_mode` all bring the localizer up afresh beside it — and the safe one for a crash: it
//! can forget a confirmation, never invent one.

/// The localizer's latest `localization_status`, as much of it as the machine reads.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Localization {
    /// The loaded map's name; empty when the localizer runs with no map.
    pub map: String,
    /// The guess queued last has converged (`relocalize_check`'s answer). What a confirmation is
    /// allowed on — NOT a running quality signal: it stays true through a lost stretch.
    pub converged: bool,
    /// Names the estimate. Any other value than the one a confirmation was given at means the
    /// estimate is not the one that was confirmed.
    pub epoch: u64,
}

/// What latched ERROR. One variant today.
///
/// ERROR has more than one cause by design: a fault reported by syncai_driver_manager is to be
/// the second (it has no such signal yet). When it comes it is a variant here, a subscription,
/// and a `raise` beside [`Lifecycle::end_task`] — the latch, the refusals and the release do not
/// change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultSource {
    /// A task ended FAILED.
    Task,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Fault {
    pub source: FaultSource,
    /// The task's id, for the log. Not shown to the operator.
    pub task_id: String,
    /// The reporter's own sentence (the failed step's error). May be empty.
    pub reason: String,
}

/// The two states an operator can ask for. Everything else is derived.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// "The robot is where the map shows it."
    Idle,
    /// Withdraw the confirmation.
    Uninitialized,
}

/// Why a confirmation was refused. Each is a different thing for the operator to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NotConverged {
    /// No `localization_status` has ever arrived: the localizer is not up — which is every
    /// moment of the mapping session, where none runs.
    NoLocalizer,
    /// The localizer runs with no map.
    NoMap,
    /// A guess is pending, or the last one was withdrawn without converging.
    GuessNotConverged,
}

/// What an operator's switch let go of, for the log line.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Released {
    pub fault: Option<Fault>,
    /// A task still counted as running: its `end_task` never came (a workflow terminated from
    /// outside), or the operator switched under a live one — which the backend refuses, and this
    /// node cannot tell apart.
    pub task: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Begun {
    /// The task is running from now on.
    Started,
    /// This task id was already running: the caller is retrying a call whose answer it lost.
    AlreadyRunning,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BeginRefusal {
    /// A failure is latched. Every task, whatever it needs.
    ErrorLatched(Fault),
    /// The task drives on the map and the position is not confirmed.
    NotInitialized,
    /// Another task is running.
    Busy { running: String },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    Completed,
    Failed,
    Canceled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ended {
    /// The task is no longer counted as running; nothing else changed.
    Closed,
    /// The same, and ERROR is latched.
    Latched,
    /// The robot is not running that task. Nothing changed.
    NotRunning,
}

#[derive(Debug, Default)]
pub struct Lifecycle {
    /// The `pose_epoch` an operator confirmed the position at, NOT a flag.
    ///
    /// Stored as the epoch so that the confirmation is withdrawn by comparison rather than by an
    /// event: a map switch, a new initial pose, an auto-localize and a restarted localizer all
    /// change the epoch, and none of them has to be noticed here. A flag cleared by a callback
    /// would be one missed or reordered sample away from surviving the thing it should not.
    confirmed_epoch: Option<u64>,
    fault: Option<Fault>,
    task: Option<String>,
    localization: Option<Localization>,
}

impl Lifecycle {
    /// Take a `localization_status` sample. Returns true when this sample withdrew a
    /// confirmation, for the caller to log — nothing else needs doing, [`Self::is_confirmed`]
    /// already answers no.
    pub fn accept_localization(&mut self, sample: Localization) -> bool {
        let was_confirmed = self.is_confirmed();
        self.localization = Some(sample);
        was_confirmed && !self.is_confirmed()
    }

    pub fn localization(&self) -> Option<&Localization> {
        self.localization.as_ref()
    }

    /// An operator has confirmed the estimate the localizer holds NOW.
    pub fn is_confirmed(&self) -> bool {
        match (self.confirmed_epoch, &self.localization) {
            (Some(confirmed), Some(localization)) => confirmed == localization.epoch,
            _ => false,
        }
    }

    /// There is an estimate an operator could confirm.
    pub fn is_converged(&self) -> bool {
        self.converged().is_ok()
    }

    fn converged(&self) -> Result<u64, NotConverged> {
        match &self.localization {
            None => Err(NotConverged::NoLocalizer),
            Some(localization) if localization.map.is_empty() => Err(NotConverged::NoMap),
            Some(localization) if !localization.converged => Err(NotConverged::GuessNotConverged),
            Some(localization) => Ok(localization.epoch),
        }
    }

    pub fn fault(&self) -> Option<&Fault> {
        self.fault.as_ref()
    }

    pub fn task(&self) -> Option<&str> {
        self.task.as_deref()
    }

    /// The operator's switch. The only way into "confirmed" and the only way out of ERROR.
    ///
    /// Either target releases a latched fault and drops a task still counted as running. The
    /// second is on purpose and is why nothing here refuses while a task runs: a workflow
    /// terminated from outside never sends its `end_task`, and without this the robot would
    /// answer BUSY for good. Refusing the switch under a live task is the backend's job — it has
    /// the job list, this node has only what it was told.
    ///
    /// A refusal changes nothing: asking for IDLE on an unconverged robot does not release an
    /// error on the way to being refused.
    pub fn set_state(&mut self, target: Target) -> Result<Released, NotConverged> {
        match target {
            Target::Idle => self.confirmed_epoch = Some(self.converged()?),
            Target::Uninitialized => self.confirmed_epoch = None,
        }
        Ok(Released {
            fault: self.fault.take(),
            task: self.task.take(),
        })
    }

    /// May this task start? An acceptance is also the transition: the task is running on return.
    ///
    /// `initialized` is the rollup's own answer (confirmed, and a pose to report), passed in so
    /// that the gate and the published `state` cannot disagree.
    ///
    /// The retry is recognised before anything else is judged: a caller whose first answer was
    /// lost must get the same one, or it reads BUSY against itself and — having been told no —
    /// never sends the `end_task` that would let the robot go.
    ///
    /// Of the refusals, the latched error comes first because it is the one that holds for every
    /// task, then the unconfirmed position (it stays until someone acts), then BUSY (it passes by
    /// itself): the reason given is the one the operator most needs.
    pub fn begin_task(
        &mut self,
        task_id: &str,
        needs_localization: bool,
        initialized: bool,
    ) -> Result<Begun, BeginRefusal> {
        if self.task.as_deref() == Some(task_id) {
            return Ok(Begun::AlreadyRunning);
        }
        if let Some(fault) = &self.fault {
            return Err(BeginRefusal::ErrorLatched(fault.clone()));
        }
        if needs_localization && !initialized {
            return Err(BeginRefusal::NotInitialized);
        }
        if let Some(running) = &self.task {
            return Err(BeginRefusal::Busy {
                running: running.clone(),
            });
        }
        self.task = Some(task_id.to_owned());
        Ok(Begun::Started)
    }

    /// Close a task `begin_task` accepted. Matched by id, a no-op otherwise.
    ///
    /// The match is what makes a repeated report harmless, and it is why a FAILED report for a
    /// task this node does not know latches nothing: after a restart mid-task the robot is
    /// UNINITIALIZED with no task, and the run it forgot must not turn that into an ERROR nobody
    /// can explain from this node's log.
    ///
    /// Completed and Canceled change nothing but the task: a robot that was unconfirmed while it
    /// stood up is unconfirmed afterwards. "Back to IDLE" is the rollup's doing, not a write.
    pub fn end_task(&mut self, task_id: &str, outcome: Outcome, reason: &str) -> Ended {
        if self.task.as_deref() != Some(task_id) {
            return Ended::NotRunning;
        }
        self.task = None;
        match outcome {
            Outcome::Completed | Outcome::Canceled => Ended::Closed,
            Outcome::Failed => {
                self.fault = Some(Fault {
                    source: FaultSource::Task,
                    task_id: task_id.to_owned(),
                    reason: reason.trim().to_owned(),
                });
                Ended::Latched
            }
        }
    }

    /// One sentence for the operator saying why the robot is ERROR or UNINITIALIZED, or that a
    /// task runs; empty when the state's own name says it all.
    ///
    /// Shown verbatim by the console, so it names no node, topic, service or id. `pose_available`
    /// is the TF lookup's answer, the rollup's other half of "initialized".
    pub fn message(&self, pose_available: bool) -> String {
        if let Some(fault) = &self.fault {
            // Irrefutable while there is one source, and so a compile error the day a second
            // is added: a driver fault must not be reported to the operator as a failed task.
            let FaultSource::Task = fault.source;
            return if fault.reason.is_empty() {
                "A task failed. Release the error before the robot takes another task.".to_owned()
            } else {
                format!(
                    "A task failed: {} Release the error before the robot takes another task.",
                    with_full_stop(&fault.reason)
                )
            };
        }
        if !self.is_confirmed() || !pose_available {
            return match self.converged() {
                Err(NotConverged::NoLocalizer) => {
                    "The robot has no position estimate yet.".to_owned()
                }
                Err(NotConverged::NoMap) => "No map is loaded.".to_owned(),
                Err(NotConverged::GuessNotConverged) => {
                    "The robot has not found its position on the map. Set an initial pose."
                        .to_owned()
                }
                Ok(_) if !pose_available => "The robot has no position estimate yet.".to_owned(),
                Ok(_) => "Waiting for an operator to confirm the robot's position.".to_owned(),
            };
        }
        if self.task.is_some() {
            return "A task is running.".to_owned();
        }
        String::new()
    }
}

impl NotConverged {
    /// The refusal an operator reads when Confirm is turned down.
    pub fn sentence(self) -> &'static str {
        match self {
            NotConverged::NoLocalizer => {
                "The position cannot be confirmed: the robot has no position estimate yet."
            }
            NotConverged::NoMap => "The position cannot be confirmed: no map is loaded.",
            NotConverged::GuessNotConverged => {
                "The position cannot be confirmed: the robot has not found its position on the \
                 map. Set an initial pose first."
            }
        }
    }
}

/// A reason is somebody else's text and may or may not end its sentence; the message goes on
/// after it.
fn with_full_stop(reason: &str) -> String {
    if reason.ends_with(['.', '!', '?']) {
        reason.to_owned()
    } else {
        format!("{reason}.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn located(epoch: u64, converged: bool) -> Localization {
        Localization {
            map: "office".to_owned(),
            converged,
            epoch,
        }
    }

    /// A machine whose position an operator confirmed at epoch 7.
    fn confirmed() -> Lifecycle {
        let mut lifecycle = Lifecycle::default();
        lifecycle.accept_localization(located(7, true));
        lifecycle.set_state(Target::Idle).unwrap();
        lifecycle
    }

    #[test]
    fn it_starts_unconfirmed_with_nothing_latched() {
        let lifecycle = Lifecycle::default();
        assert!(!lifecycle.is_confirmed());
        assert!(!lifecycle.is_converged());
        assert_eq!(lifecycle.fault(), None);
        assert_eq!(lifecycle.task(), None);
    }

    #[test]
    fn a_converged_guess_confirms_nothing_by_itself() {
        let mut lifecycle = Lifecycle::default();
        assert!(!lifecycle.accept_localization(located(7, true)));
        assert!(lifecycle.is_converged());
        assert!(!lifecycle.is_confirmed());
    }

    #[test]
    fn confirming_needs_a_converged_estimate() {
        let mut lifecycle = Lifecycle::default();
        assert_eq!(
            lifecycle.set_state(Target::Idle),
            Err(NotConverged::NoLocalizer)
        );

        lifecycle.accept_localization(Localization {
            map: String::new(),
            converged: false,
            epoch: 1,
        });
        assert_eq!(lifecycle.set_state(Target::Idle), Err(NotConverged::NoMap));

        lifecycle.accept_localization(located(2, false));
        assert_eq!(
            lifecycle.set_state(Target::Idle),
            Err(NotConverged::GuessNotConverged)
        );
        assert!(!lifecycle.is_confirmed());

        lifecycle.accept_localization(located(2, true));
        assert_eq!(lifecycle.set_state(Target::Idle), Ok(Released::default()));
        assert!(lifecycle.is_confirmed());
    }

    #[test]
    fn another_epoch_withdraws_the_confirmation() {
        let mut lifecycle = confirmed();
        // The same estimate reported again (a late joiner's replay) withdraws nothing.
        assert!(!lifecycle.accept_localization(located(7, true)));
        assert!(lifecycle.is_confirmed());

        // A new guess: the epoch moves before the guess has converged...
        assert!(lifecycle.accept_localization(located(8, false)));
        assert!(!lifecycle.is_confirmed());
        // ...and its converging does not give the confirmation back.
        assert!(!lifecycle.accept_localization(located(8, true)));
        assert!(!lifecycle.is_confirmed());
    }

    #[test]
    fn a_guess_that_converged_between_two_samples_still_withdraws() {
        // Queued and converged before anything was published in between: `converged` never
        // read false, and only the epoch says the estimate is another one.
        let mut lifecycle = confirmed();
        assert!(lifecycle.accept_localization(located(8, true)));
        assert!(!lifecycle.is_confirmed());
    }

    #[test]
    fn the_operator_can_withdraw_a_confirmation() {
        let mut lifecycle = confirmed();
        assert_eq!(
            lifecycle.set_state(Target::Uninitialized),
            Ok(Released::default())
        );
        assert!(!lifecycle.is_confirmed());
    }

    #[test]
    fn a_task_that_drives_needs_a_confirmed_position() {
        let mut lifecycle = Lifecycle::default();
        lifecycle.accept_localization(located(7, true));
        assert_eq!(
            lifecycle.begin_task("move", true, false),
            Err(BeginRefusal::NotInitialized)
        );
        // Refused, so nothing is running and nothing is latched.
        assert_eq!(lifecycle.task(), None);
        assert_eq!(lifecycle.fault(), None);
        // Standing up needs no position.
        assert_eq!(
            lifecycle.begin_task("stand", false, false),
            Ok(Begun::Started)
        );
        assert_eq!(lifecycle.task(), Some("stand"));
    }

    #[test]
    fn one_task_at_a_time_and_a_retry_is_not_a_second_task() {
        let mut lifecycle = confirmed();
        assert_eq!(lifecycle.begin_task("a", true, true), Ok(Begun::Started));
        assert_eq!(
            lifecycle.begin_task("a", true, true),
            Ok(Begun::AlreadyRunning)
        );
        assert_eq!(
            lifecycle.begin_task("b", true, true),
            Err(BeginRefusal::Busy {
                running: "a".to_owned()
            })
        );
        assert_eq!(lifecycle.task(), Some("a"));
    }

    #[test]
    fn a_retry_is_recognised_even_after_the_confirmation_went() {
        // The estimate changed under a running task. Its own retried begin must still read
        // "running", or the caller is told no and never closes it.
        let mut lifecycle = confirmed();
        lifecycle.begin_task("a", true, true).unwrap();
        lifecycle.accept_localization(located(8, false));
        assert_eq!(
            lifecycle.begin_task("a", true, false),
            Ok(Begun::AlreadyRunning)
        );
    }

    #[test]
    fn finishing_or_cancelling_changes_nothing_but_the_task() {
        for outcome in [Outcome::Completed, Outcome::Canceled] {
            let mut lifecycle = confirmed();
            lifecycle.begin_task("a", true, true).unwrap();
            assert_eq!(lifecycle.end_task("a", outcome, ""), Ended::Closed);
            assert_eq!(lifecycle.task(), None);
            assert_eq!(lifecycle.fault(), None);
            assert!(lifecycle.is_confirmed());
        }
    }

    #[test]
    fn a_posture_task_on_an_unconfirmed_robot_does_not_confirm_it() {
        let mut lifecycle = Lifecycle::default();
        lifecycle.accept_localization(located(7, true));
        lifecycle.begin_task("stand", false, false).unwrap();
        assert_eq!(
            lifecycle.end_task("stand", Outcome::Completed, ""),
            Ended::Closed
        );
        assert!(!lifecycle.is_confirmed());
    }

    #[test]
    fn a_failed_task_latches_until_the_operator_releases_it() {
        let mut lifecycle = confirmed();
        lifecycle.begin_task("a", true, true).unwrap();
        assert_eq!(
            lifecycle.end_task("a", Outcome::Failed, " goal occupied "),
            Ended::Latched
        );
        let fault = Fault {
            source: FaultSource::Task,
            task_id: "a".to_owned(),
            reason: "goal occupied".to_owned(),
        };
        assert_eq!(lifecycle.fault(), Some(&fault));

        // Every task, the ones that need no position included.
        assert_eq!(
            lifecycle.begin_task("b", true, true),
            Err(BeginRefusal::ErrorLatched(fault.clone()))
        );
        assert_eq!(
            lifecycle.begin_task("stand", false, true),
            Err(BeginRefusal::ErrorLatched(fault.clone()))
        );
        // A refusal is not a failure: still one fault, still the first.
        assert_eq!(lifecycle.fault(), Some(&fault));

        assert_eq!(
            lifecycle.set_state(Target::Idle),
            Ok(Released {
                fault: Some(fault),
                task: None
            })
        );
        assert_eq!(lifecycle.fault(), None);
        assert_eq!(lifecycle.begin_task("b", true, true), Ok(Begun::Started));
    }

    #[test]
    fn an_error_can_be_released_without_confirming() {
        let mut lifecycle = confirmed();
        lifecycle.begin_task("a", true, true).unwrap();
        lifecycle.end_task("a", Outcome::Failed, "x");
        assert!(lifecycle.set_state(Target::Uninitialized).is_ok());
        assert_eq!(lifecycle.fault(), None);
        assert!(!lifecycle.is_confirmed());
    }

    #[test]
    fn a_refused_confirmation_releases_nothing() {
        let mut lifecycle = confirmed();
        lifecycle.begin_task("a", true, true).unwrap();
        lifecycle.end_task("a", Outcome::Failed, "x");
        lifecycle.accept_localization(located(8, false));
        assert_eq!(
            lifecycle.set_state(Target::Idle),
            Err(NotConverged::GuessNotConverged)
        );
        assert!(lifecycle.fault().is_some());
    }

    #[test]
    fn the_switch_drops_a_task_whose_end_never_came() {
        let mut lifecycle = confirmed();
        lifecycle.begin_task("gone", true, true).unwrap();
        assert_eq!(
            lifecycle.set_state(Target::Idle),
            Ok(Released {
                fault: None,
                task: Some("gone".to_owned())
            })
        );
        assert_eq!(lifecycle.begin_task("next", true, true), Ok(Begun::Started));
        // The stray report, if it ever comes, is for a task this robot is not running.
        assert_eq!(
            lifecycle.end_task("gone", Outcome::Failed, "x"),
            Ended::NotRunning
        );
        assert_eq!(lifecycle.fault(), None);
        assert_eq!(lifecycle.task(), Some("next"));
    }

    #[test]
    fn a_report_for_an_unknown_task_changes_nothing() {
        let mut lifecycle = confirmed();
        assert_eq!(
            lifecycle.end_task("never-begun", Outcome::Failed, "x"),
            Ended::NotRunning
        );
        assert_eq!(lifecycle.fault(), None);

        // And a report repeated after it was taken is the same no-op.
        lifecycle.begin_task("a", true, true).unwrap();
        assert_eq!(
            lifecycle.end_task("a", Outcome::Completed, ""),
            Ended::Closed
        );
        assert_eq!(
            lifecycle.end_task("a", Outcome::Failed, "x"),
            Ended::NotRunning
        );
        assert_eq!(lifecycle.fault(), None);
    }

    #[test]
    fn the_message_says_what_the_robot_is_waiting_for() {
        let mut lifecycle = Lifecycle::default();
        assert_eq!(
            lifecycle.message(false),
            "The robot has no position estimate yet."
        );

        lifecycle.accept_localization(Localization {
            map: String::new(),
            converged: false,
            epoch: 1,
        });
        assert_eq!(lifecycle.message(true), "No map is loaded.");

        lifecycle.accept_localization(located(2, false));
        assert_eq!(
            lifecycle.message(true),
            "The robot has not found its position on the map. Set an initial pose."
        );

        lifecycle.accept_localization(located(2, true));
        assert_eq!(
            lifecycle.message(true),
            "Waiting for an operator to confirm the robot's position."
        );

        lifecycle.set_state(Target::Idle).unwrap();
        assert_eq!(lifecycle.message(true), "");

        lifecycle.begin_task("a", true, true).unwrap();
        assert_eq!(lifecycle.message(true), "A task is running.");

        lifecycle.end_task("a", Outcome::Failed, "The goal is occupied");
        assert_eq!(
            lifecycle.message(true),
            "A task failed: The goal is occupied. Release the error before the robot takes \
             another task."
        );
    }

    #[test]
    fn a_failure_with_no_reason_still_reads_as_a_sentence() {
        let mut lifecycle = confirmed();
        lifecycle.begin_task("a", true, true).unwrap();
        lifecycle.end_task("a", Outcome::Failed, "  ");
        assert_eq!(
            lifecycle.message(true),
            "A task failed. Release the error before the robot takes another task."
        );
    }
}
