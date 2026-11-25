//! The three services of the robot's state machine, on the node's single worker.
//!
//! Names are relative, so they land on `<robot_id>/set_robot_state`, `<robot_id>/begin_task` and
//! `<robot_id>/end_task`. syncai_backend is the caller of all three and is out of tree, so each
//! name and each .srv is a cross-repository contract: `begin_task` / `end_task` bracket every
//! task run (a refusal fails the run before its first step), and `set_robot_state` is what the
//! console's Confirm control reaches.
//!
//! They share the worker the subscriptions and the publish timer run on, and that is the whole
//! concurrency story: a `begin_task` cannot interleave with a `localization_status` sample or
//! with another `begin_task`, so "is the position confirmed, and is nothing else running" is
//! judged and applied without a lock. It is also why the answers are immediate — nothing a
//! handler does blocks, each is a few comparisons and a publish.

use rclrs::*;
use ros_env::syncai_common::srv::{
    BeginTask, BeginTask_Request, EndTask, EndTask_Request, SetRobotState, SetRobotState_Request,
};

use super::state::RobotStateData;

pub struct Services {
    _set_robot_state: WorkerService<SetRobotState, RobotStateData>,
    _begin_task: WorkerService<BeginTask, RobotStateData>,
    _end_task: WorkerService<EndTask, RobotStateData>,
}

impl Services {
    pub fn create(worker: &Worker<RobotStateData>) -> Result<Self, RclrsError> {
        Ok(Self {
            _set_robot_state: worker.create_service::<SetRobotState, _>(
                "set_robot_state",
                |data: &mut RobotStateData, request: SetRobotState_Request| {
                    data.set_robot_state(request)
                },
            )?,
            _begin_task: worker.create_service::<BeginTask, _>(
                "begin_task",
                |data: &mut RobotStateData, request: BeginTask_Request| data.begin_task(request),
            )?,
            _end_task: worker.create_service::<EndTask, _>(
                "end_task",
                |data: &mut RobotStateData, request: EndTask_Request| data.end_task(request),
            )?,
        })
    }
}
