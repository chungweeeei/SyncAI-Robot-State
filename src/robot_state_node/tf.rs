//! The TF lookup the C++ node gets from `tf2_ros::Buffer` + `TransformListener`.
//!
//! rclrs has no `tf2_ros` binding, so the node subscribes `/tf` and `/tf_static` itself and feeds
//! the transforms into [`TfBuffer`]. [`TfBuffer::pose_of`] then answers the one question this node
//! ever asks: where is `base_frame` in `global_frame`, at the latest available time.
//!
//! Nothing here touches ROS types — the subscription converts at the boundary — so the whole
//! module is unit-testable without a ROS environment or the robot, like `protocol.rs` in
//! syncai_driver_manager.
//!
//! # What this deliberately does not do
//!
//! * **No time history, no interpolation.** Only the newest transform per child frame is kept.
//!   The C++ looks up at `tf2::TimePointZero`, which tf2 resolves to the newest sample anyway, so
//!   a history would never be read.
//! * **No separate static cache.** tf2 stores `/tf_static` in a cache that never expires; this
//!   buffer expires nothing at all, so the distinction has nothing to do. Both subscriptions feed
//!   one map and the last publisher of a given child frame wins, as in tf2.
//! * **No cache expiry.** tf2 drops entries older than its 10 s cache window but never drops the
//!   newest one for a frame, so a frozen publisher leaves a stale transform in place there too.
//!
//! That last point has a consequence the C++ version shares and does not state: once
//! `map -> base_link` has been seen ONCE, `localization_valid` stays true forever, even if the
//! localizer dies and the pose freezes. `transform_tolerance` does not catch it in either version
//! — see the note on that parameter in `parameters.rs`.

use std::collections::HashMap;
use std::fmt;

/// How far up the tree a walk may go before the tree is declared malformed. TF is a tree, so a
/// real chain is a handful of links; only a publisher that made a frame its own ancestor can
/// exceed this, and without the bound that walk would not terminate.
const MAX_CHAIN_DEPTH: usize = 100;

/// How far a quaternion's norm may stray from 1 before the transform is rejected. Same tolerance
/// tf2's `assertQuaternionValid` uses.
const QUATERNION_NORM_TOLERANCE: f64 = 1e-6;

/// A quaternion in the x/y/z/w order `geometry_msgs/Quaternion` uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    /// The identity rotation — NOT all zeroes, which is not a rotation at all.
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

impl Quaternion {
    /// Hamilton product, i.e. "apply `rhs` first, then `self`".
    fn multiply(self, rhs: Self) -> Self {
        Self {
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        }
    }

    /// The inverse rotation. Only correct for a unit quaternion, which `TfBuffer::insert` is what
    /// guarantees: it rejects anything whose norm is not 1.
    fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        // q * (0, v) * q^-1, expanded so no temporary quaternion is built
        let u = [self.x, self.y, self.z];
        let uv = cross(u, v);
        let uuv = cross(u, uv);
        [
            v[0] + 2.0 * (self.w * uv[0] + uuv[0]),
            v[1] + 2.0 * (self.w * uv[1] + uuv[1]),
            v[2] + 2.0 * (self.w * uv[2] + uuv[2]),
        ]
    }

    fn norm(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    /// The Z component of the ZYX Euler decomposition, matching `tf2::getYaw`.
    ///
    /// The degenerate branch is tf2's: at a pitch of +/-90 degrees yaw and roll describe the same
    /// rotation, and tf2 resolves that by putting all of it in roll and reporting a yaw of 0. A
    /// quadruped on the ground never reaches it, but reproducing it keeps this a port rather than
    /// an approximation.
    pub fn yaw(self) -> f64 {
        let m20 = 2.0 * (self.x * self.z - self.w * self.y);
        if m20.abs() >= 1.0 {
            return 0.0;
        }
        let m10 = 2.0 * (self.x * self.y + self.w * self.z);
        let m00 = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        m10.atan2(m00)
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// A rigid transform, read as `parent <- child`: the child frame expressed in the parent's
/// coordinates. That is exactly what one `geometry_msgs/TransformStamped` carries, with
/// `header.frame_id` the parent and `child_frame_id` the child.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Transform {
    pub translation: [f64; 3],
    pub rotation: Quaternion,
}

impl Transform {
    /// Compose: `self` is `A <- B` and `rhs` is `B <- C`, so the result is `A <- C`.
    fn then(self, rhs: Self) -> Self {
        let t = self.rotation.rotate(rhs.translation);
        Self {
            translation: [
                self.translation[0] + t[0],
                self.translation[1] + t[1],
                self.translation[2] + t[2],
            ],
            rotation: self.rotation.multiply(rhs.rotation),
        }
    }

    /// `A <- B` becomes `B <- A`.
    fn inverse(self) -> Self {
        let rotation = self.rotation.conjugate();
        let t = rotation.rotate(self.translation);
        Self {
            translation: [-t[0], -t[1], -t[2]],
            rotation,
        }
    }
}

/// What the node takes from a transform: a position plus a heading, which is all
/// `RobotLocalizationStatus.position` carries.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Radians.
    pub yaw: f64,
}

/// Why a transform was not stored. The caller logs it; the buffer keeps its previous value for
/// that frame, the same way tf2's `setTransform` returns false and changes nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidTransform {
    /// `header.frame_id == child_frame_id`. A frame cannot be its own parent.
    SelfParent,
    /// Either frame name is empty.
    EmptyFrameId,
    /// A NaN or an infinity somewhere in the translation or the rotation.
    NonFinite,
    /// The rotation is not a unit quaternion — including the all-zero one a default-constructed
    /// `geometry_msgs/Quaternion` carries, which is the common way this fires.
    NotUnitQuaternion,
}

impl fmt::Display for InvalidTransform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = match self {
            Self::SelfParent => "frame_id equals child_frame_id",
            Self::EmptyFrameId => "frame_id or child_frame_id is empty",
            Self::NonFinite => "translation or rotation contains NaN/inf",
            Self::NotUnitQuaternion => "rotation is not a unit quaternion",
        };
        f.write_str(text)
    }
}

/// Why a lookup failed. All three mean the same thing to the node — `localization_valid = false`
/// — but they say very different things to whoever reads the log.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LookupError {
    /// Nothing has ever been published about this frame. On this stack it is the normal state
    /// before `syncai_localizer` has been relocalized: `map` exists, `map -> odom` does not.
    UnknownFrame(String),
    /// Both frames are known but sit in trees that do not meet — usually a half-built TF graph.
    NotConnected { target: String, source: String },
    /// A frame is its own ancestor. Only a misbehaving publisher produces this.
    CycleDetected(String),
}

impl fmt::Display for LookupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownFrame(frame) => write!(f, "frame '{frame}' is unknown to TF"),
            Self::NotConnected { target, source } => {
                write!(f, "no TF path from '{source}' to '{target}'")
            }
            Self::CycleDetected(frame) => {
                write!(f, "TF tree has a cycle at or above '{frame}'")
            }
        }
    }
}

struct Link {
    parent: String,
    /// `parent <- child`, where the child is the map key.
    transform: Transform,
}

/// The newest transform per child frame, and the tree walk over them.
#[derive(Default)]
pub struct TfBuffer {
    links: HashMap<String, Link>,
}

impl TfBuffer {
    /// Store one transform, replacing whatever was last heard for `child`.
    ///
    /// A frame has exactly one parent in TF, so the child is the key. Two publishers claiming the
    /// same child fight over this slot — which is a broken TF graph, and tf2 behaves the same way.
    pub fn insert(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform,
    ) -> Result<(), InvalidTransform> {
        if parent.is_empty() || child.is_empty() {
            return Err(InvalidTransform::EmptyFrameId);
        }
        if parent == child {
            return Err(InvalidTransform::SelfParent);
        }
        let r = transform.rotation;
        if !transform.translation.iter().all(|v| v.is_finite())
            || ![r.x, r.y, r.z, r.w].iter().all(|v| v.is_finite())
        {
            return Err(InvalidTransform::NonFinite);
        }
        if (r.norm() - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
            return Err(InvalidTransform::NotUnitQuaternion);
        }

        self.links.insert(
            child.to_owned(),
            Link {
                parent: parent.to_owned(),
                transform,
            },
        );
        Ok(())
    }

    /// Whether anything has ever been published about this frame, as a parent or as a child.
    fn knows(&self, frame: &str) -> bool {
        self.links.contains_key(frame) || self.links.values().any(|link| link.parent == frame)
    }

    /// Walk up to the root, returning it together with `root <- frame`.
    /// The returned name is either `frame` itself (when nothing is published about its parent) or
    /// a borrow out of the buffer, so both have to share one lifetime.
    fn chain_to_root<'a>(&'a self, frame: &'a str) -> Result<(&'a str, Transform), LookupError> {
        let mut current = frame;
        // Starts as `frame <- frame`, and grows into `current <- frame` as the walk goes up
        let mut to_frame = Transform::default();

        for _ in 0..MAX_CHAIN_DEPTH {
            let Some(link) = self.links.get(current) else {
                return Ok((current, to_frame));
            };
            // `parent <- current` composed with `current <- frame` gives `parent <- frame`
            to_frame = link.transform.then(to_frame);
            current = &link.parent;
        }

        Err(LookupError::CycleDetected(frame.to_owned()))
    }

    /// `target <- source`: the source frame expressed in the target's coordinates.
    ///
    /// This is what `tf2_ros::Buffer::lookupTransform(target, source, TimePointZero)` returns, and
    /// equivalently what the C++ node gets by transforming an identity pose in `base_frame` into
    /// `global_frame`.
    pub fn lookup(&self, target: &str, source: &str) -> Result<Transform, LookupError> {
        if !self.knows(target) {
            return Err(LookupError::UnknownFrame(target.to_owned()));
        }
        if !self.knows(source) {
            return Err(LookupError::UnknownFrame(source.to_owned()));
        }
        if target == source {
            return Ok(Transform::default());
        }

        let (target_root, to_target) = self.chain_to_root(target)?;
        let (source_root, to_source) = self.chain_to_root(source)?;
        if target_root != source_root {
            return Err(LookupError::NotConnected {
                target: target.to_owned(),
                source: source.to_owned(),
            });
        }

        // (root <- target)^-1 composed with (root <- source) is (target <- source). Going via the
        // root rather than the lowest common ancestor gives the same answer: the extra links
        // above the common ancestor appear once in each chain and cancel.
        Ok(to_target.inverse().then(to_source))
    }

    /// [`Self::lookup`] reduced to the position and heading `RobotState` carries.
    pub fn pose_of(&self, target: &str, source: &str) -> Result<Pose, LookupError> {
        let transform = self.lookup(target, source)?;
        Ok(Pose {
            x: transform.translation[0],
            y: transform.translation[1],
            z: transform.translation[2],
            yaw: transform.rotation.yaw(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A rotation of `yaw` radians about Z.
    fn yaw_quaternion(yaw: f64) -> Quaternion {
        Quaternion {
            x: 0.0,
            y: 0.0,
            z: (yaw / 2.0).sin(),
            w: (yaw / 2.0).cos(),
        }
    }

    fn transform(translation: [f64; 3], yaw: f64) -> Transform {
        Transform {
            translation,
            rotation: yaw_quaternion(yaw),
        }
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_pose_close(actual: Pose, expected: Pose) {
        assert_close(actual.x, expected.x);
        assert_close(actual.y, expected.y);
        assert_close(actual.z, expected.z);
        assert_close(actual.yaw, expected.yaw);
    }

    #[test]
    fn yaw_round_trips_through_the_quaternion() {
        for yaw in [0.0, 0.5, -0.5, 1.5, -1.5, 3.0, -3.0] {
            assert_close(yaw_quaternion(yaw).yaw(), yaw);
        }
    }

    #[test]
    fn identity_quaternion_has_zero_yaw() {
        assert_close(Quaternion::default().yaw(), 0.0);
    }

    #[test]
    fn a_single_link_is_returned_as_published() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 2.0, 0.5], 0.75))
            .unwrap();

        assert_pose_close(
            buffer.pose_of("map", "base_link").unwrap(),
            Pose {
                x: 1.0,
                y: 2.0,
                z: 0.5,
                yaw: 0.75,
            },
        );
    }

    /// The real chain on this stack: syncai_localizer publishes map -> odom, syncai_lio_bridge
    /// publishes odom -> base_link, and the node asks for map -> base_link.
    #[test]
    fn a_two_link_chain_composes_rotation_and_translation() {
        let mut buffer = TfBuffer::default();
        // map -> odom: a quarter turn, no offset
        buffer
            .insert(
                "map",
                "odom",
                transform([0.0, 0.0, 0.0], std::f64::consts::FRAC_PI_2),
            )
            .unwrap();
        // odom -> base_link: 2 m along odom's x, which the quarter turn swings onto map's y
        buffer
            .insert("odom", "base_link", transform([2.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_pose_close(
            buffer.pose_of("map", "base_link").unwrap(),
            Pose {
                x: 0.0,
                y: 2.0,
                z: 0.0,
                yaw: std::f64::consts::FRAC_PI_2,
            },
        );
    }

    #[test]
    fn a_lookup_against_the_walk_direction_is_the_inverse() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert(
                "map",
                "base_link",
                transform([1.0, 2.0, 0.0], std::f64::consts::FRAC_PI_2),
            )
            .unwrap();

        let there = buffer.lookup("map", "base_link").unwrap();
        let back = buffer.lookup("base_link", "map").unwrap();
        let round_trip = there.then(back);

        assert_close(round_trip.translation[0], 0.0);
        assert_close(round_trip.translation[1], 0.0);
        assert_close(round_trip.translation[2], 0.0);
        assert_close(round_trip.rotation.yaw(), 0.0);
    }

    /// The two frames meet below the root, so the links above the meeting point must cancel.
    #[test]
    fn a_lookup_between_two_branches_goes_through_the_common_ancestor() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "odom", transform([5.0, 5.0, 0.0], 1.0))
            .unwrap();
        buffer
            .insert("odom", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("odom", "lidar", transform([0.0, 3.0, 0.0], 0.0))
            .unwrap();

        assert_pose_close(
            buffer.pose_of("base_link", "lidar").unwrap(),
            Pose {
                x: -1.0,
                y: 3.0,
                z: 0.0,
                yaw: 0.0,
            },
        );
    }

    #[test]
    fn a_frame_is_at_the_origin_of_itself() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 2.0, 3.0], 1.0))
            .unwrap();

        assert_eq!(buffer.pose_of("map", "map").unwrap(), Pose::default());
    }

    /// The pre-relocalize state on the 3D stack: odom -> base_link is being published, map -> odom
    /// is not, so `map` has never been mentioned.
    #[test]
    fn an_unseen_frame_is_reported_as_unknown() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("odom", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_eq!(
            buffer.pose_of("map", "base_link"),
            Err(LookupError::UnknownFrame("map".into()))
        );
    }

    #[test]
    fn two_separate_trees_do_not_connect() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "odom", transform([0.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("other_map", "base_link", transform([0.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_eq!(
            buffer.pose_of("map", "base_link"),
            Err(LookupError::NotConnected {
                target: "map".into(),
                source: "base_link".into(),
            })
        );
    }

    #[test]
    fn a_cycle_is_detected_rather_than_walked_forever() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("a", "b", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("b", "a", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_eq!(
            buffer.pose_of("a", "b"),
            Err(LookupError::CycleDetected("a".into()))
        );
    }

    #[test]
    fn the_newest_transform_for_a_frame_replaces_the_previous_one() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("map", "base_link", transform([9.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_close(buffer.pose_of("map", "base_link").unwrap().x, 9.0);
    }

    #[test]
    fn malformed_transforms_are_rejected_and_change_nothing() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();

        let zero_quaternion = Transform {
            translation: [7.0, 0.0, 0.0],
            rotation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
        };
        assert_eq!(
            buffer.insert("map", "base_link", zero_quaternion),
            Err(InvalidTransform::NotUnitQuaternion)
        );
        assert_eq!(
            buffer.insert("map", "map", transform([0.0, 0.0, 0.0], 0.0)),
            Err(InvalidTransform::SelfParent)
        );
        assert_eq!(
            buffer.insert("", "base_link", transform([0.0, 0.0, 0.0], 0.0)),
            Err(InvalidTransform::EmptyFrameId)
        );
        assert_eq!(
            buffer.insert("map", "base_link", transform([f64::NAN, 0.0, 0.0], 0.0)),
            Err(InvalidTransform::NonFinite)
        );

        // The good transform from the top is still the one that answers
        assert_close(buffer.pose_of("map", "base_link").unwrap().x, 1.0);
    }
}
