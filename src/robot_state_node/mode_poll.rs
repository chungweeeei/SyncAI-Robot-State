//! `RobotState.mode` — which byobu session is up, polled from syncai_sys_manager.
//!
//! Deliberately a service poll and not a subscription: sys_manager DERIVES the mode by asking
//! byobu which session exists and never stores it, so it is the only party that can answer, and it
//! publishes no mode topic. Adding one there would mean a second source of truth to keep honest.
//!
//! The previous implementation hardcoded AUTO, which made the frontend's mode chip a lie whenever
//! a mapping session was live.

use rclrs::*;
use ros_env::syncai_common::msg::RobotMode;
use ros_env::syncai_common::srv::{GetMode, GetMode_Request, GetMode_Response};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

/// How many poll ticks an unanswered request may sit before it is presumed dead and a fresh one is
/// sent. At the default 1 Hz that is 5 s — far longer than a healthy sys_manager needs (get_mode is
/// two `byobu has-session` subprocesses), but short enough to recover from a sys_manager restart
/// within a few polls instead of waiting forever on a reply that will never come.
const STALE_TICKS: u32 = 5;

const LOG_THROTTLE: Duration = Duration::from_secs(10);

/// For log lines only — the message carries the numeric constant.
fn mode_name(mode: u8) -> &'static str {
    match mode {
        RobotMode::MAINTENANCE => "MAINTENANCE",
        RobotMode::MANUAL => "MANUAL",
        RobotMode::AUTO => "AUTO",
        _ => "UNKNOWN",
    }
}

/// Numbers one request after another, so a reply can be matched to the request it answers.
///
/// Needed because rclrs has no `prune_pending_requests`: a request this node has given up on
/// stays alive inside the client, and its reply — should it arrive after all — would otherwise
/// be indistinguishable from the reply to the request that replaced it.
type Generation = u64;

/// What one poll tick should do, decided under the lock so the decision and the bookkeeping that
/// backs it cannot be split by the response callback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TickAction {
    /// A request is in flight and has not yet been waited on for long enough to presume it dead.
    Wait,
    /// Send a request tagged with `generation`. `abandoned_after` is the number of polls the
    /// previous request went unanswered when this one replaces it, for the log.
    Send {
        generation: Generation,
        abandoned_after: Option<u32>,
    },
}

/// What became of a reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Reply {
    /// The reply answers the request in flight; `previous` is the mode it replaces.
    Adopted { previous: u8 },
    /// The reply answers a request that was already abandoned, so it was ignored: the request
    /// that replaced it is still in flight and its answer is fresher by construction.
    Stale,
}

/// Shared between the poll tick (which runs on the node's worker) and the service response
/// callback (which runs on the executor's task pool) — the one place in this port that needs a
/// lock, and the same one the C++ version takes for `reported_mode_`.
///
/// The transitions are plain methods on plain fields, so the one invariant that matters — at
/// most one request is ever considered in flight, and only its own reply can release it — is
/// unit-tested without a client or a service.
#[derive(Debug)]
struct Cache {
    /// Starts at AUTO, not MAINTENANCE: this node only ever runs inside a session that sys_manager
    /// built or adopted, so "no answer yet" almost always means "the 1 Hz poll has not fired",
    /// not "the stack is down" — and AUTO is also exactly what the field reported for its entire
    /// hardcoded life. In a mapping session it reads AUTO for at most one poll period before
    /// correcting itself to MANUAL.
    reported: u8,
    /// The request currently in flight, if any. At most one: without this a sys_manager mid-switch
    /// (its switch handler holds a lock across ~40 byobu commands, and get_mode shares the node)
    /// would accumulate one queued request per tick, all answering at once when it frees up.
    in_flight: Option<Generation>,
    /// Polls the request in flight has gone unanswered.
    stale_ticks: u32,
    next_generation: Generation,
}

impl Default for Cache {
    fn default() -> Self {
        Self {
            reported: RobotMode::AUTO,
            in_flight: None,
            stale_ticks: 0,
            next_generation: 0,
        }
    }
}

impl Cache {
    /// One poll tick: either keep waiting on the request in flight, or claim the next generation
    /// for a new one. Claiming it here, before anything is sent, is what keeps a second tick from
    /// sending too.
    fn begin_request(&mut self) -> TickAction {
        let abandoned_after = if self.in_flight.is_some() {
            self.stale_ticks += 1;
            if self.stale_ticks < STALE_TICKS {
                return TickAction::Wait;
            }
            // Presumed dead — e.g. sys_manager restarted between our send and its reply, so the
            // response will never arrive. Fall through and send a fresh request.
            Some(self.stale_ticks)
        } else {
            None
        };

        let generation = self.next_generation;
        self.next_generation += 1;
        self.in_flight = Some(generation);
        self.stale_ticks = 0;
        TickAction::Send {
            generation,
            abandoned_after,
        }
    }

    /// The reply to `generation` arrived carrying `mode`.
    fn reply(&mut self, generation: Generation, mode: u8) -> Reply {
        if self.in_flight != Some(generation) {
            return Reply::Stale;
        }
        self.in_flight = None;
        Reply::Adopted {
            previous: std::mem::replace(&mut self.reported, mode),
        }
    }

    /// The request tagged `generation` never went out, so nothing is in flight after all.
    fn release(&mut self, generation: Generation) {
        if self.in_flight == Some(generation) {
            self.in_flight = None;
        }
    }
}

pub struct ModePoll {
    client: Client<GetMode>,
    logger: Logger,
    cache: Arc<Mutex<Cache>>,
}

impl ModePoll {
    pub fn create(node: &Node) -> Result<Self, RclrsError> {
        Ok(Self {
            // Relative name, so it lands on <robot_id>/get_mode next to sys_manager's server
            client: node.create_client::<GetMode>("get_mode")?,
            logger: node.logger().clone(),
            cache: Arc::default(),
        })
    }

    /// The last answer, or AUTO if none has arrived. Up to one poll period stale by construction;
    /// that is the accepted cost of not spamming sys_manager with subprocess-backed service calls
    /// at the publish rate.
    pub fn reported_mode(&self) -> u8 {
        lock(&self.cache).reported
    }

    /// One poll. Non-blocking by construction: a readiness check, then at most one async request.
    pub fn tick(&self) {
        match self.client.service_is_ready() {
            Ok(true) => {}
            Ok(false) => {
                // An absent sys_manager is a normal condition to ride out — it restarts
                // independently of the sessions it manages — so the cached mode simply holds,
                // the same policy as every sample cache in this node. Waiting for the service
                // would block the worker instead.
                log_warn!(
                    self.logger.throttle(LOG_THROTTLE),
                    "[ModePoll] get_mode service unavailable; holding mode {}",
                    mode_name(self.reported_mode()),
                );
                return;
            }
            Err(e) => {
                log_warn!(
                    self.logger.throttle(LOG_THROTTLE),
                    "[ModePoll] could not check whether get_mode is ready: {e}"
                );
                return;
            }
        }

        let TickAction::Send {
            generation,
            abandoned_after,
        } = lock(&self.cache).begin_request()
        else {
            return;
        };
        if let Some(polls) = abandoned_after {
            // Difference from the C++ version: rclrs has no `prune_pending_requests`, so the
            // abandoned callback stays in the client's request board rather than being released
            // here. It costs one closure per lost response, which only a reply lost mid-flight
            // produces — while sys_manager is simply down, `service_is_ready` above stops us
            // sending at all. Should that reply turn up after all, `Cache::reply` recognises it
            // by its generation and ignores it.
            log_warn!(
                &self.logger,
                "[ModePoll] get_mode unanswered for {polls} polls; retrying"
            );
        }

        let cache = Arc::clone(&self.cache);
        let logger = self.logger.clone();
        let request = GetMode_Request::default();
        let sent = self
            .client
            .call_then(&request, move |response: GetMode_Response| {
                // Adopted even when success is false: per GetMode.srv, false means AMBIGUOUS
                // (both sessions exist, possible only if they were built by hand) and `mode`
                // then holds the first match — still the best available answer, and better
                // than freezing the field on a staler one.
                let Reply::Adopted { previous } = lock(&cache).reply(generation, response.mode)
                else {
                    log_warn!(
                        logger.throttle(LOG_THROTTLE),
                        "[ModePoll] ignoring a late get_mode reply to a request already given \
                         up on (mode {})",
                        mode_name(response.mode),
                    );
                    return;
                };

                if !response.success {
                    log_warn!(
                        logger.throttle(LOG_THROTTLE),
                        "[ModePoll] get_mode reports an ambiguous mode: {}",
                        response.message,
                    );
                }
                if previous != response.mode {
                    log_info!(
                        &logger,
                        "[ModePoll] mode {} -> {} (session '{}')",
                        mode_name(previous),
                        mode_name(response.mode),
                        response.session,
                    );
                }
            });

        if let Err(e) = sent {
            // The request never went out, so release the guard or every later tick would see a
            // request in flight that does not exist and wait STALE_TICKS for nothing.
            lock(&self.cache).release(generation);
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[ModePoll] failed to send a get_mode request: {e}"
            );
        }
    }
}

/// A poisoned lock here means a previous callback panicked while holding it. The cache is four
/// plain fields with no invariant a panic could have broken halfway, and refusing to report a mode
/// forever is worse than carrying on, so the guard is recovered rather than propagated.
fn lock(cache: &Mutex<Cache>) -> MutexGuard<'_, Cache> {
    cache
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(cache: &mut Cache) -> Generation {
        match cache.begin_request() {
            TickAction::Send { generation, .. } => generation,
            TickAction::Wait => panic!("expected a request to be sent"),
        }
    }

    #[test]
    fn the_first_tick_sends_and_the_reply_is_adopted() {
        let mut cache = Cache::default();
        assert_eq!(cache.reported, RobotMode::AUTO);

        let generation = send(&mut cache);
        assert_eq!(
            cache.reply(generation, RobotMode::MANUAL),
            Reply::Adopted {
                previous: RobotMode::AUTO
            }
        );
        assert_eq!(cache.reported, RobotMode::MANUAL);
        assert_eq!(cache.in_flight, None);
    }

    #[test]
    fn only_one_request_is_in_flight_while_a_reply_is_pending() {
        let mut cache = Cache::default();
        let generation = send(&mut cache);
        for _ in 1..STALE_TICKS {
            assert_eq!(cache.begin_request(), TickAction::Wait);
        }
        assert_eq!(cache.in_flight, Some(generation));
    }

    #[test]
    fn an_unanswered_request_is_replaced_after_stale_ticks() {
        let mut cache = Cache::default();
        let first = send(&mut cache);
        for _ in 1..STALE_TICKS {
            assert_eq!(cache.begin_request(), TickAction::Wait);
        }

        let TickAction::Send {
            generation: second,
            abandoned_after,
        } = cache.begin_request()
        else {
            panic!("expected a retry");
        };
        assert_ne!(first, second);
        assert_eq!(abandoned_after, Some(STALE_TICKS));
        assert_eq!(cache.in_flight, Some(second));
        assert_eq!(cache.stale_ticks, 0);
    }

    /// The reason the generation exists: a late reply to an abandoned request must neither
    /// release the guard (or the next tick would put a second request in flight) nor overwrite
    /// the mode (its replacement's answer is fresher).
    #[test]
    fn a_late_reply_to_an_abandoned_request_is_ignored() {
        let mut cache = Cache::default();
        let first = send(&mut cache);
        for _ in 1..STALE_TICKS {
            cache.begin_request();
        }
        let second = send(&mut cache);

        assert_eq!(cache.reply(first, RobotMode::MAINTENANCE), Reply::Stale);
        assert_eq!(cache.reported, RobotMode::AUTO);
        assert_eq!(cache.in_flight, Some(second));
        assert_eq!(cache.begin_request(), TickAction::Wait);

        assert_eq!(
            cache.reply(second, RobotMode::MANUAL),
            Reply::Adopted {
                previous: RobotMode::AUTO
            }
        );
        assert_eq!(cache.in_flight, None);
    }

    #[test]
    fn a_reply_that_arrives_twice_is_adopted_once() {
        let mut cache = Cache::default();
        let generation = send(&mut cache);
        assert!(matches!(
            cache.reply(generation, RobotMode::MANUAL),
            Reply::Adopted { .. }
        ));
        assert_eq!(
            cache.reply(generation, RobotMode::MAINTENANCE),
            Reply::Stale
        );
        assert_eq!(cache.reported, RobotMode::MANUAL);
    }

    #[test]
    fn a_failed_send_releases_the_guard_immediately() {
        let mut cache = Cache::default();
        let generation = send(&mut cache);
        cache.release(generation);
        assert_eq!(cache.in_flight, None);
        assert!(matches!(cache.begin_request(), TickAction::Send { .. }));
    }

    /// Releasing a generation that is no longer the one in flight must not knock out its
    /// replacement.
    #[test]
    fn releasing_a_superseded_generation_changes_nothing() {
        let mut cache = Cache::default();
        let first = send(&mut cache);
        for _ in 1..STALE_TICKS {
            cache.begin_request();
        }
        let second = send(&mut cache);

        cache.release(first);
        assert_eq!(cache.in_flight, Some(second));
    }
}
