# Launch the syncai_robot_state node — publishes syncai_common/RobotState at 1 Hz, with position
# from TF (map -> <robot_id>/base_link, its height through <robot_id>/pointlio_body and
# <robot_id>/lidar_top_imu) and forward velocity from the odom topic.
#
# robot_id is read from the system config INI at launch time (same convention as
# sys_manager.launch.py) and is used as the node namespace, as the `robot_id` node parameter, and
# to rewrite the base_frame / body_frame / lidar_frame parameters, since TF frame names are not
# namespaced by ROS. The global_frame stays "map".
#
# The INI's [system] robot_model (g23 when absent) picks params/models/<robot_model>.yaml, loaded
# after robot_state_params.yaml: that model's six input topics and its lidar_frame, the URDF link
# the height chain closes on (2026-11-24, the layout syncai_driver_manager has). A model with no
# such file, or one that leaves out a key, fails the launch: the code defaults are the G23's, and
# another robot on them would look healthy with half its state missing.
#
# The same INI names the loaded map in [map] name; the launch passes it as the `map` node parameter,
# so the published RobotState.map is the map this instance loaded, without editing
# robot_state_params.yaml. It used to read [map] map (a gridmap.yaml path, from map_server's era);
# the workspace dropped that key on 2026-10-28 with 2D navigation, after which every robot
# published the params file's stale default (dp2f_full) whatever map it had loaded.

import configparser
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute path so the INI resolves no matter what cwd the launch is started from. ~/robot_ws is
# the workspace inside the robot container, where docker-compose bind-mounts the per-robot instance
# INI over config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

# Every INI before [system] robot_model existed is a G23's (the workspace's launches read the key
# the same way).
DEFAULT_ROBOT_MODEL = "g23"

# What a model's file must set. An empty topic is a deliberate "not subscribed"; a missing key
# would silently be the G23's.
MODEL_KEYS = (
    "odom_topic",
    "battery_state_topic",
    "wifi_status_topic",
    "motor_states_topic",
    "mode_topic",
    "safety_locked_topic",
    "lidar_frame",
)

logger = launch_logging.get_logger("robot_state.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def read_robot_model(config_path: str) -> str:
    """[system] robot_model of the INI, `g23` when the key (or the file) is missing."""
    config = configparser.ConfigParser()
    config.read(config_path)
    model = config.get("system", "robot_model", fallback="").strip().lower()
    if not model:
        logger.warning(
            f"No [system] robot_model in '{config_path}'; reading it as '{DEFAULT_ROBOT_MODEL}'"
        )
        return DEFAULT_ROBOT_MODEL
    return model


def model_params(model: str) -> tuple[str, dict]:
    """params/models/<model>.yaml in this package's share, and its parameters.

    Raises when the file is missing, unreadable, or leaves out one of MODEL_KEYS. A new model's
    file needs a colcon build of this package before a launch finds it (--symlink-install links
    each file, not the directory).
    """
    path = os.path.join(
        get_package_share_directory("syncai_robot_state"), "params", "models", f"{model}.yaml"
    )
    if not os.path.isfile(path):
        raise RuntimeError(
            f"robot_model '{model}' has no params/models/{model}.yaml in syncai_robot_state; "
            "add one (see params/models/g23.yaml) and rebuild, or fix [system] robot_model"
        )
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = (data.get("/**") or {}).get("ros__parameters") or {}
    except (OSError, yaml.YAMLError, AttributeError) as e:
        raise RuntimeError(f"robot_model '{model}': '{path}' is unreadable ({e})") from e
    missing = [k for k in MODEL_KEYS if k not in params]
    if missing:
        raise RuntimeError(
            f"robot_model '{model}': '{path}' leaves {missing} unset under /** "
            "(the code defaults are the G23's)"
        )
    skipped = [k for k in MODEL_KEYS if k.endswith("_topic") and not params[k]]
    logger.info(
        f"robot_model '{model}': {path}"
        + (f"; not subscribed: {', '.join(skipped)}" if skipped else "")
    )
    return path, params


def read_map(config_path: str) -> str | None:
    """The loaded map's bare name from the INI, as the workspace's launch files and the backend
    read it.

    [map] name first. An older INI with no `name` key falls back to the parent directory of its
    [map] map path (map/dp2f/gridmap.yaml -> dp2f), the backend's same fallback. An INI that names
    no map (an empty `name:` is a robot with no map yet, the first deployment's INI) returns "":
    RobotState.map is then empty rather than some other map's name. None only when the INI cannot
    be read, which leaves the params-file `map` in place.
    """
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return None

    name = config.get("map", "name", fallback="").strip()
    if not name:
        yaml_path = config.get("map", "map", fallback="").strip()
        if yaml_path:
            name = os.path.basename(os.path.dirname(yaml_path))

    if not name:
        logger.info(f"No [map] name in '{config_path}'; publishing an empty RobotState.map")
        return ""

    logger.info(f"Map from '{config_path}': {name}")
    return name


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration values only resolve inside an OpaqueFunction, and we need the resolved
    # robot_id here to namespace the node and its frames.
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    map_value = read_map(config_path)
    model_file, model = model_params(read_robot_model(config_path))

    params_file = LaunchConfiguration("params_file")

    # Override the yaml defaults with the INI robot_id. TF frame names are not namespaced by ROS,
    # so the three robot frames get the robot_id prefix here. Later entries in this list take
    # precedence over the params file.
    overrides = {
        "robot_id": robot_id,
        "base_frame": f"{robot_id}/base_link",
        "body_frame": f"{robot_id}/pointlio_body",
        # The model's link that closes the body chain back onto the base, prefixed here like the
        # other frames (the G23's lidar_top_imu, its lidar's IMU; the M20's lidar_link). Written
        # into the launch until 2026-11-24.
        "lidar_frame": f"{robot_id}/{model['lidar_frame']}",
    }
    # An INI with no map overrides too (with ""), so a stale params-file value is never published
    # for a robot that has none; only an unreadable INI leaves the params file's value.
    if map_value is not None:
        overrides["map"] = map_value

    # No `name=`: the node is constructed as "syncai_robot_state", and the params file uses a bare
    # /** key (rclrs does not expand /**/syncai_robot_state-style wildcards), so it matches in any
    # namespace.
    robot_state_node = Node(
        package="syncai_robot_state",
        executable="robot_state_node",
        namespace=robot_id,
        output="screen",
        # All three under /**: one key, so a later entry wins key by key. The model's file sets
        # none of the overrides' keys but lidar_frame, which the overrides prefix.
        parameters=[
            params_file,
            model_file,
            overrides,
        ],
    )

    return [robot_state_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_robot_state")
    default_params_file = os.path.join(pkg_share, "params", "robot_state_params.yaml")

    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id and robot_model",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the robot_state parameters YAML file",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_params_file,
            OpaqueFunction(function=launch_setup),
        ]
    )
