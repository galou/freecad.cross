# Load trajectories from ROS bag

## Context
`Cross::Robot` can already load trajectories from a multi-doc YAML file (`robot_proxy.py:1139`, `load_trajectories_from_yaml`). We need the same for ROS 2 bags, so recorded data (e.g. `/robocon_controller/joint_states` in `/tmp/claude/measurements_on_robot/2026_10_09/rosbag2_*`) can be replayed on the robot model.

Environment facts I checked:
- `rosbag2_py`, `rclpy.serialization` and `rosidl_runtime_py.utilities.get_message` work with ROS Jazzy on Python 3.12. FreeCAD's pixi Python is also 3.12, and the `freecad` launcher passes `PYTHONPATH` on with `--python-path`.
- `mcap` / `mcap_ros2` are not installed, so I use `rosbag2_py`.

## Decisions (agreed with the user)
- `sensor_msgs/msg/JointState`: 1 Trajectory per topic, 1 point per message. A joint missing from a message repeats its last value. No decimation.
- `moveit_msgs/msg/DisplayTrajectory`: 1 Trajectory per RobotTrajectory, in every message.
- `trajectory_msgs/msg/JointTrajectory`: 1 Trajectory per message.
- Fix both bugs in `update_trajectory`: the unknown-joint KeyError, and the stale `prop_name` in the start-state loop (`trajectory_proxy.py:232-237`).

## Implementation steps
1. **New `freecad/cross/rosbag_utils.py`**. ROS imports are behind an import guard (`try/except ImportError` sets a `_g_imports_ok` flag, as in `trajectory_proxy.py:19-28`). It contains:
   - `COMPATIBLE_TYPES` = the 3 types above.
   - `find_bag(path) -> (bag_dir, metadata: dict) | None`: if `path` is a directory, use `path/metadata.yaml`. If it is `*.yaml`/`*.yml`, use the file itself. For any other file (mcap or unknown), use `metadata.yaml` next to it. The metadata is parsed with `yaml.safe_load`, so no ROS is needed. Returns None with `warn(...)` if the file is missing or invalid.
   - `compatible_topics(metadata) -> list[(name, type, count)]` from `rosbag2_bagfile_information.topics_with_message_count`.
   - `read_messages(bag_dir, storage_id, topics) -> dict[topic, list[msg]]`: `SequentialReader` + `StorageFilter(topics=...)` + `deserialize_message(get_message(type))`, in one pass. The storage id comes from the metadata (`storage_identifier`).
   -> verify: run it with system `python3` on `rosbag2_2026_10_09-10_28_34`. Expect 1 compatible topic and 820 JointState messages.
2. **`trajectory_proxy.py`**: add message loaders that build on `update_trajectory`:
   - `load_joint_trajectory_msg(msg)`: calls `update_trajectory(msg)`.
   - `load_robot_trajectory_msg(robot_traj, start_state)`: calls `update_trajectory(robot_traj.joint_trajectory, start_state)`, used for DisplayTrajectory.
   - `load_joint_states(msgs)`: builds a `JointTrajectory` (joint names = union in first-seen order, last-value fill) and calls `update_trajectory`.
   - Fix in `update_trajectory`: skip with a warning when `n not in self._joint_map` (currently `self._joint_map[n]` raises KeyError before the `hasattr` check).
   - Fix in `update_trajectory`: the start-state loop must use `self._joint_map.get(n)` instead of the leftover `prop_name`, and skip unknown joints with a warning.
   - Each loader warns and returns if `not _g_imports_ok`.
3. **New `freecad/cross/ui/choose_topics_dialog.py`**: a small QDialog built in code (no .ui). It has a checkable `QListWidget` with rows of the form `name [type] (N msgs)`, all checked by default, plus OK/Cancel. `exec()` returns the list of selected topic names, or an empty list.
4. **`robot_proxy.py`**:
   - In `setupContextMenu` (`:1112`), add the action "Load trajectories from ROS bag..." connected to `load_trajectories_from_rosbag(vobj)`.
   - GUI method: check `rosbag_utils._g_imports_ok` first and warn nicely ("ROS 2 (rosbag2_py) not available, source ROS before starting FreeCAD"). Then a `QFileDialog` in `ExistingFile` mode with filters `ROS bag (metadata.yaml *.yaml *.mcap *.db3);;All files (*)`. A directory has to be openable too: use a non-native dialog that accepts directories, or a "select directory" button. I'll pick the simplest option that works in FreeCAD when I test. Then `find_bag`, then `ChooseTopicsDialog`, then the non-GUI part.
   - Non-GUI `load_trajectories_from_bag(robot, bag_dir, metadata, topics) -> list[CrossTrajectory]`, for MCP testing. It reads the messages and creates objects with `make_trajectory(f'Trajectory_{topic}', doc)`, sets `Robot`, and calls the matching loader. It then recomputes, selects the new objects and runs `Std_ToggleFreeze` like the YAML path.
   - Late imports, as in the YAML loader.
5. When implementation starts, save this plan as `docs/plans/PLAN-2026-10-14-load-trajectories-from-rosbag.md`.

## Verification
1. Unit check of `rosbag_utils` with system python3 on all 4 test bags. Test each input kind: bag directory, `metadata.yaml`, `.mcap` file, and an unknown file in the bag dir.
2. Start `freecad` in the background with ROS sourced. In FreeCAD MCP (`execute_code`), import `robocon_primitives.urdf` as a Cross robot, then call `load_trajectories_from_bag(...)` on `rosbag2_2026_10_09-10_28_34`. Check that one `Trajectory` is created whose joint properties have 820 values, and that setting `PointIndex` moves the robot (screenshot with `get_view`).
3. Import guard: in FreeCAD, simulate `_g_imports_ok = False` and check that a clean warning appears with no traceback.
4. The dialog-driven path (context menu → file dialog → topic dialog) is left for the user to click through. No bag with DisplayTrajectory or JointTrajectory topics is available, so those loaders get only a synthetic-message check (messages built in Python and passed to the loaders).
