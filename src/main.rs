mod robot_state_node;

use rclrs::*;

use robot_state_node::RobotStateNode;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // rclrs inverts the rclcpp order: the executor comes first, and the node is created from it.
    // There is no equivalent of the C++ main's "hold the node in a named variable" trap — the node
    // here is an owned handle, not something the executor keeps a weak_ptr to.
    let mut executor = Context::default_from_env()?.create_basic_executor();
    // The node name the C++ version uses, so this stays a drop-in replacement: the params file
    // matches on it, and `ros2 node list` must not change.
    let node = executor.create_node("syncai_robot_state")?;
    let _robot_state = RobotStateNode::new(node)?;

    // No SIGINT handler: rclrs does not install one, so Ctrl-C ends the process without running
    // Drop. In scripts, stop a backgrounded node with SIGTERM.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
