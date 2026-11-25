mod health;
mod lifecycle;
mod mode_poll;
mod parameters;
mod services;
mod state;
mod subscribers;
mod tf;
mod wifi;

use std::error::Error;

use rclrs::*;
use ros_env::syncai_common::msg::RobotState;

use mode_poll::ModePoll;
use parameters::Parameters;
use services::Services;
use state::RobotStateData;
use subscribers::Subscriptions;

/// Aggregates the robot's scattered status sources into one `syncai_common/RobotState`, published
/// on the relative topic `robot_state` (so it lands on `<robot_id>/robot_state`) — and, since
/// 2026-11, owns the robot's state machine and answers whether a task may start.
///
/// One publisher, one timer, three services, and syncai_backend as the only consumer and caller
/// — which is out of tree since 2026-09, so the topic and the services are a cross-repository
/// contract: widening a field is free, changing or removing one is not.
///
/// Most of the message is still a straight copy: a yaw extraction, the height through the lidar,
/// a unit conversion, the TF-presence flag and two battery latches. The exception is `state`,
/// which until 2026-11 was derived from those and now rolls up [`lifecycle::Lifecycle`]: the
/// operator's confirmation of the position, the running task, and a latched failure. Through
/// `begin_task` that is a decision other software obeys — the first thing this node does that is
/// not a report. No threshold here commands the robot to do anything, still; refusing a task is
/// the whole of it.
///
/// # Threading
///
/// Everything runs on ONE rclrs [`Worker`]: the nine subscriptions, the three services, the
/// publish timer and the get_mode poll timer. Callbacks on a worker never overlap, which gives the
/// sample caches, the TF buffer, the health latches and the state machine exactly the protection the C++ version's `mutex_` gives them, with
/// no lock to take.
///
/// The C++ gives its 10 Hz timer a second, `MutuallyExclusive` callback group, and that is not
/// needed here. The reason it exists there is that `tf2_ros::Buffer::transform()` BLOCKS for the
/// whole `transform_tolerance` whenever the transform is absent — which it is for as long as the
/// localizer has not been relocalized — so the tick had to be kept off the thread serving the
/// sensor callbacks. This port looks the pose up in its own in-memory [`tf::TfBuffer`], which is a
/// few hash lookups and never blocks, and the mode poll is a readiness check plus an async send.
/// With nothing left to block on, a second worker would buy a lock and nothing else.
///
/// The one exception is the `get_mode` response callback, which the executor runs on its own task
/// pool rather than on the worker; it reaches a small `Mutex` inside [`ModePoll`] and nothing else.
pub struct RobotStateNode {
    // Fields drop in declaration order: the timers stop firing before the subscriptions and the
    // worker that their callbacks run on go away.
    _timers: Timers,
    _services: Services,
    _subscriptions: Subscriptions,
    _worker: Worker<RobotStateData>,
    // Parameters are undeclared when their handles drop, so they are kept for the node's life
    _parameters: Parameters,
    _node: Node,
}

impl RobotStateNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let (parameters, config) = Parameters::declare(&node)?;
        let publish_period = config.publish_period;
        let mode_poll_period = config.mode_poll_period;

        // Relative name, so it lands on <robot_id>/robot_state and stays inside this robot's
        // namespace like every other topic in the stack.
        //
        // Single publisher with latest-value semantics, so depth 1 is enough. BEST_EFFORT matches
        // odom / battery_state / wifi_status and the nature of a periodic snapshot — but note the
        // consequence for anyone writing a new subscriber: a best-effort publisher cannot satisfy
        // a RELIABLE subscriber, so subscribing with the rclcpp/rclpy default QoS receives
        // NOTHING.
        let publisher: Publisher<RobotState> =
            node.create_publisher("robot_state".keep_last(1).best_effort().volatile())?;

        let worker = node.create_worker(RobotStateData::new(
            config,
            publisher,
            node.get_clock(),
            node.logger().clone(),
            ModePoll::create(&node)?,
        ));

        let subscriptions = Subscriptions::create(&worker)?;
        let services = Services::create(&worker)?;
        let timers = Timers::create(&worker, publish_period, mode_poll_period)?;

        log_info!(
            node.logger(),
            "[RobotStateNode] Robot State initialized successfully"
        );

        Ok(Self {
            _timers: timers,
            _services: services,
            _subscriptions: subscriptions,
            _worker: worker,
            _parameters: parameters,
            _node: node,
        })
    }
}

struct Timers {
    _publish: WorkerTimer<RobotStateData>,
    _mode_poll: WorkerTimer<RobotStateData>,
}

impl Timers {
    fn create(
        worker: &Worker<RobotStateData>,
        publish_period: std::time::Duration,
        mode_poll_period: std::time::Duration,
    ) -> Result<Self, RclrsError> {
        Ok(Self {
            _publish: worker.create_timer_repeating(
                publish_period,
                |data: &mut RobotStateData| {
                    data.on_timer();
                },
            )?,
            // Shares the worker with the publish tick, unlike the C++ version where this timer is
            // deliberately kept out of the timer callback group. There the point was that a slow
            // poll must never delay a publish; here the poll cannot be slow — it checks readiness
            // and sends asynchronously, and the answer arrives on the executor's task pool.
            _mode_poll: worker.create_timer_repeating(
                mode_poll_period,
                |data: &mut RobotStateData| {
                    data.poll_mode();
                },
            )?,
        })
    }
}
