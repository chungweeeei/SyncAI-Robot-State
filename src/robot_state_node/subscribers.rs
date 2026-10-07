//! The eight subscriptions, all on the node's single worker.
//!
//! Topic names are relative so they inherit the `<robot_id>` namespace — except `/tf` and
//! `/tf_static`, which are absolute because TF is global in ROS 2. That is why the launch file
//! prefixes the FRAME name (`<robot_id>/base_link`) instead: frames are how this stack separates
//! robots that share one `/tf`, since the topic itself cannot be.
//!
//! The QoS of each one mirrors its publisher exactly. Getting one wrong is silent — a reliable
//! subscriber simply never matches a best-effort publisher and receives nothing at all — so check
//! with `ros2 topic info /<robot_id>/<topic> --verbose` rather than by reading the code.

use rclrs::*;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::BatteryState;
use ros_env::std_msgs::msg::{Bool, Int32MultiArray};
use ros_env::syncai_common::msg::{MotorStates, WifiStatus};
use ros_env::tf2_msgs::msg::TFMessage;

use super::state::RobotStateData;
use super::tf::{Quaternion, Transform};
use super::wifi::WifiInfo;

/// tf2's own listener QoS: depth 100 and reliable, with `/tf_static` additionally transient-local
/// so the static transforms published before this node started are still delivered. Without the
/// transient-local the static part of the tree would be missing until something republished it,
/// which static publishers by definition do not do.
const TF_DEPTH: u32 = 100;

pub struct Subscriptions {
    _odom: WorkerSubscription<Odometry, RobotStateData>,
    _battery_state: WorkerSubscription<BatteryState, RobotStateData>,
    _wifi_status: WorkerSubscription<WifiStatus, RobotStateData>,
    _motor_states: WorkerSubscription<MotorStates, RobotStateData>,
    _mode: WorkerSubscription<Int32MultiArray, RobotStateData>,
    _safety_locked: WorkerSubscription<Bool, RobotStateData>,
    _tf: WorkerSubscription<TFMessage, RobotStateData>,
    _tf_static: WorkerSubscription<TFMessage, RobotStateData>,
}

impl Subscriptions {
    pub fn create(worker: &Worker<RobotStateData>) -> Result<Self, RclrsError> {
        Ok(Self {
            // From syncai_lio_bridge
            _odom: worker.create_subscription(
                "odom".sensor_data_qos(),
                |data: &mut RobotStateData, msg: Odometry| {
                    data.accept_odom(msg.twist.twist.linear.x);
                },
            )?,

            // From syncai_driver_manager
            _battery_state: worker.create_subscription(
                "battery_state".sensor_data_qos(),
                |data: &mut RobotStateData, msg: BatteryState| {
                    data.accept_battery(msg.percentage);
                },
            )?,

            // From syncai_sys_manager
            _wifi_status: worker.create_subscription(
                "wifi_status".keep_last(1).best_effort().volatile(),
                |data: &mut RobotStateData, msg: WifiStatus| {
                    data.accept_wifi(WifiInfo {
                        ssid: msg.ssid,
                        bssid: msg.bssid,
                        rssi: msg.rssi,
                        ip_address: msg.ip_address,
                        mac_address: msg.mac_address,
                    });
                },
            )?,

            // From syncai_driver_manager. SensorData to match its publisher.
            _motor_states: worker.create_subscription(
                "motor_states".sensor_data_qos(),
                |data: &mut RobotStateData, msg: MotorStates| {
                    data.accept_motor_states(msg);
                },
            )?,

            // The gait controller's own state machine, as syncai_driver_manager reports it back.
            // RELIABLE depth 10 — an exact mirror of that publisher.
            //
            // Reliability is REQUESTED rather than merely tolerated because this topic is
            // edge-triggered, not a periodic stream: the driver publishes only when a telemetry
            // datagram happens to carry a MODE_STATE section, with no periodic republish and no
            // transient-local latch. A dropped sample is therefore not made good by the next one —
            // it can be the only announcement of a state change.
            //
            // The cost of the choice, stated so it is not a surprise later: if that publisher is
            // ever relaxed to best-effort this subscription stops matching and receives NOTHING,
            // silently.
            _mode: worker.create_subscription(
                "mode".keep_last(10),
                |data: &mut RobotStateData, msg: Int32MultiArray| {
                    // `layout` is ignored: the driver never populates it (it assigns data only),
                    // so the positional meaning of data[] is convention, not something the message
                    // declares.
                    data.accept_low_level_mode(&msg.data);
                },
            )?,

            // syncai_driver_manager's software safety lock (its `SafetyLock`), carried on
            // RobotState.low_level_mode.safety_state. RELIABLE + TRANSIENT_LOCAL depth 1, an exact
            // mirror of that publisher, which is latched: it publishes the released state once at
            // startup and then only on a change.
            //
            // The transient-local is what makes this work at all. With nothing periodic behind the
            // topic, a VOLATILE subscriber started after the driver (the normal boot order — this
            // pane comes up last) would never hear the startup sample and would report `false`
            // until the lock next moved, which could be never.
            //
            // The price is that this reader now REQUIRES a transient-local writer. DDS matches
            // durability only when the offered level is at least the requested one, so if that
            // publisher is ever relaxed to VOLATILE this subscription stops matching SILENTLY and
            // safety_state reads `false` for good — not merely without the late-joiner delivery.
            // (The reverse pairing, a volatile reader on a transient-local writer, is the one that
            // is compatible.) Change the two together.
            _safety_locked: worker.create_subscription(
                "safety_locked".keep_last(1).reliable().transient_local(),
                |data: &mut RobotStateData, msg: Bool| {
                    data.accept_safety_locked(msg.data);
                },
            )?,

            _tf: worker.create_subscription(
                "/tf".keep_last(TF_DEPTH),
                |data: &mut RobotStateData, msg: TFMessage| {
                    accept_tf_message(data, msg);
                },
            )?,
            _tf_static: worker.create_subscription(
                "/tf_static".keep_last(TF_DEPTH).transient_local(),
                |data: &mut RobotStateData, msg: TFMessage| {
                    accept_tf_message(data, msg);
                },
            )?,
        })
    }
}

/// Both TF topics land here: the buffer expires nothing, so it has no reason to tell static
/// transforms from dynamic ones (see `tf.rs`).
fn accept_tf_message(data: &mut RobotStateData, msg: TFMessage) {
    for stamped in &msg.transforms {
        let translation = &stamped.transform.translation;
        let rotation = &stamped.transform.rotation;
        data.accept_transform(
            strip_leading_slash(&stamped.header.frame_id),
            strip_leading_slash(&stamped.child_frame_id),
            Transform {
                translation: [translation.x, translation.y, translation.z],
                rotation: Quaternion {
                    x: rotation.x,
                    y: rotation.y,
                    z: rotation.z,
                    w: rotation.w,
                },
            },
        );
    }
}

/// ROS 2 frame ids carry no leading slash, but a bridged or hand-written ROS 1 publisher still
/// emits one. tf2 strips it for the same reason: otherwise `/map` and `map` are two frames and the
/// lookup fails with nothing in the log to explain why.
fn strip_leading_slash(frame_id: &str) -> &str {
    frame_id.strip_prefix('/').unwrap_or(frame_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ros1_style_frame_id_loses_its_slash() {
        assert_eq!(strip_leading_slash("/map"), "map");
        assert_eq!(strip_leading_slash("map"), "map");
        assert_eq!(
            strip_leading_slash("/robot01/base_link"),
            "robot01/base_link"
        );
        assert_eq!(strip_leading_slash(""), "");
    }
}
