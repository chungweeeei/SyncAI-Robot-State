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

/// Shared between the poll tick (which runs on the node's worker) and the service response
/// callback (which runs on the executor's task pool) — the one place in this port that needs a
/// lock, and the same one the C++ version takes for `reported_mode_`.
#[derive(Debug)]
struct Cache {
    /// Starts at AUTO, not MAINTENANCE: this node only ever runs inside a session that sys_manager
    /// built or adopted, so "no answer yet" almost always means "the 1 Hz poll has not fired",
    /// not "the stack is down" — and AUTO is also exactly what the field reported for its entire
    /// hardcoded life. In a mapping session it reads AUTO for at most one poll period before
    /// correcting itself to MANUAL.
    reported: u8,
    /// At most one request in flight. Without this a sys_manager mid-switch (its switch handler
    /// holds a lock across ~40 byobu commands, and get_mode shares the node) would accumulate one
    /// queued request per tick, all answering at once when it frees up.
    pending: bool,
    stale_ticks: u32,
}

impl Default for Cache {
    fn default() -> Self {
        Self {
            reported: RobotMode::AUTO,
            pending: false,
            stale_ticks: 0,
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

        {
            let mut cache = lock(&self.cache);
            if cache.pending {
                cache.stale_ticks += 1;
                if cache.stale_ticks < STALE_TICKS {
                    return;
                }
                // Presumed dead — e.g. sys_manager restarted between our send and its reply, so
                // the response will never arrive. Fall through and send a fresh request.
                //
                // Difference from the C++ version: rclrs has no `prune_pending_requests`, so the
                // abandoned callback stays in the client's request board rather than being
                // released here. It costs one closure per lost response, which only a reply lost
                // mid-flight produces — while sys_manager is simply down, `service_is_ready`
                // above stops us sending at all.
                log_warn!(
                    &self.logger,
                    "[ModePoll] get_mode unanswered for {} polls; retrying",
                    cache.stale_ticks,
                );
            }
            cache.pending = true;
            cache.stale_ticks = 0;
        }

        let cache = Arc::clone(&self.cache);
        let logger = self.logger.clone();
        let request = GetMode_Request::default();
        let sent = self
            .client
            .call_then(&request, move |response: GetMode_Response| {
                let previous = {
                    let mut cache = lock(&cache);
                    cache.pending = false;
                    let previous = cache.reported;
                    // Adopted even when success is false: per GetMode.srv, false means AMBIGUOUS
                    // (both sessions exist, possible only if they were built by hand) and `mode`
                    // then holds the first match — still the best available answer, and better
                    // than freezing the field on a staler one.
                    cache.reported = response.mode;
                    previous
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
            lock(&self.cache).pending = false;
            log_warn!(
                self.logger.throttle(LOG_THROTTLE),
                "[ModePoll] failed to send a get_mode request: {e}"
            );
        }
    }
}

/// A poisoned lock here means a previous callback panicked while holding it. The cache is three
/// plain fields with no invariant a panic could have broken halfway, and refusing to report a mode
/// forever is worse than carrying on, so the guard is recovered rather than propagated.
fn lock(cache: &Mutex<Cache>) -> MutexGuard<'_, Cache> {
    cache
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}
