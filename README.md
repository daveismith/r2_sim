# r2_sim — Simulation components for the R2 robot

This package contains the simulation-only parts of the R2 ROS 2 project: the Gazebo launch, the `gz_ros2_control` hardware description, worlds, and sim-specific nodes and configuration. It uses the same description (`r2_description`), controllers and topics as the real robot (`r2_bringup`), with `gz_ros2_control` in place of the hardware interfaces.

## Contents

- `launch/sim_foxglove.launch.py` — the sim headless, with a `foxglove_bridge` on sim time for viewing it in Foxglove.
- `launch/launch_sim.launch.py` — starts Gazebo, spawns the robot, starts the controllers, and runs the shared bringup pieces (joystick, twist_mux, dome teleop, joint states, web interface).
- `urdf/gz_ros2_control.xacro` — the sim `<ros2_control>` block and the `gz_ros2_control` plugin, included by `r2_bringup`'s `robot.urdf.xacro` when `sim_mode` is true.
- `config/sim_controllers.yaml` — sim overrides on top of `r2_bringup/config/my_controllers.yaml`, including the `servo_pos` joint list (body panels and utility arms).
- `config/dome_servos.yaml` — the dome panel joints and their speeds in Gazebo.
- `config/gz_bridge.yaml` — topics bridged between Gazebo and ROS.
- `worlds/` — simulation worlds (`obstacles.world` is the default, `empty.world`).
- `r2_sim/servo_sim.py` — servo mirror and emulator (see [Servo joints](#servo-joints)).

## Requirements

- A working ROS 2 Jazzy installation and the per-machine workspace `setup.bash` (see the `r2_bringup` README).
- `colcon` for building the workspace.
- Gazebo with `ros_gz_sim`, `ros_gz_bridge` and `gz_ros2_control`.

## Build & setup

Build from the workspace root (one level above `src/`), always with `--symlink-install`:

```bash
cd /home/davids/r2_ws
source ./setup.bash
colcon build --symlink-install --packages-select r2_sim
```

To build the entire workspace, leave out `--packages-select`. Re-source `./setup.bash` after a build that adds new packages or executables. `servo_sim` is installed as a copy rather than a symlink, so rebuild after editing it.

## Running the simulation

```bash
ros2 launch r2_sim launch_sim.launch.py
```

| argument | default | meaning |
| --- | --- | --- |
| `world` | `worlds/obstacles.world` | Gazebo world to load |
| `gui` | `true` | `false` runs the Gazebo server headless, for machines without a display |
| `servo_device` | `sim` | `sim`: emulate the dome servos the real device isn't publishing (all of them if there's no device). `real`: emulate none; only the real device drives them (see below) |

### Viewing the sim in Foxglove

Without a display (the dev container), run Gazebo headless and view the robot in Foxglove:

```bash
ros2 launch r2_sim sim_foxglove.launch.py
```

It runs `launch_sim.launch.py gui:=false` (passing through `world` and `servo_device`) and a
`foxglove_bridge` on sim time on `foxglove_port` (default 8766, since the dev container's
live-robot bridge holds 8765). Connect Foxglove to `ws://localhost:8766`, turn on
`/robot_description` in the 3D panel, set the display frame to `base_link`, and under Scene set
**Mesh up axis to Z** (R2's meshes are Z-up; Foxglove's Y-up default draws them rotated 90°).

## Servo joints

The panels and utility arms are the joints tagged `<astromech:servo>` in `r2_description`. In Gazebo they're driven in two ways:

- **Dome panels (`pp*`, `p*`):** each one has its own Gazebo `JointPositionController` in velocity mode. It moves the panel at a constant speed to its target, like the real servo, and holds it closed from spawn. The joints and their speeds are listed in `config/dome_servos.yaml`:

  ```yaml
  pp1_joint: {velocity: 16.0}   # rad/s; 16.0 opens 0 -> pi/2 in about 100 ms, like the real servos
  ```

  `urdf/gz_ros2_control.xacro` generates one controller per entry, with commands on `/servo/<joint>/cmd_pos`. The launch file bridges those topics from ROS. To change a panel's speed, edit its entry and rebuild.
- **Body panels and utility arms (`bf*`, `br*`, `*_utility_arm`):** held closed by the `servo_pos` controller (`position_controllers/JointGroupPositionController`). Its joint list in `config/sim_controllers.yaml` is the single source for these joints. On the real robot nothing drives them.

A joint must be in only one of the two lists.

On the real robot the dome panels are driven by a separate device on the network, `r2_domeplayer`. It takes commands on `/dome/joint_command` and publishes `/dome/joint_states`, both as `sensor_msgs/JointState` in radians. The sim uses the same topics:

```
/dome/joint_command ──> servo device (servo_sim emulator, or the real device)
                                │
                        /dome/joint_states ──> joint_state_publisher ──> /joint_states
                                │
                         servo_sim mirror ──> /servo/<joint>/cmd_pos (dome panels)
                                          └─> /servo_pos/commands (body, arms) ──> Gazebo
```

`servo_sim` (`r2_sim/servo_sim.py`) does two things:

- **Mirror (always on):** sends `/dome/joint_states` to Gazebo as soon as each message arrives, so Gazebo follows whatever the servo device reports. Dome panels go to their own controllers; any other reported joint goes to `servo_pos`. Joints that are never reported stay closed (0).
- **Emulate (`servo_device:=sim`):** follows the device's contract. The servos are the `<astromech:servo>` joints on the dome, taken from `/robot_description`. Commands can name a joint as `pp1` or `pp1_joint`. Positions are clamped to the URDF `<limit>`, and the clamp is logged. Unknown joints are refused (`unknown joint`); each refused joint is logged only the first time. The commanded targets are published on `/dome/joint_states` at 10 Hz, and immediately on each command.

  If a real dome device is on the network, the emulator fills in only for it. Any joint the device has reported on `/dome/joint_states` within the last second (`claim_timeout`) is left out of the emulator's messages, so each joint has exactly one source. This matters while the device has only some of its servos configured. The emulator recognises its own messages by `header.frame_id: servo_sim`; the device leaves the field empty. While a joint is muted, its target follows the device's reported position, so if the device stops reporting it, emulation resumes from there without a jump. The node logs each switch.

`/joint_states` reports the servo device's view, which is the commanded target, as on the real robot. It does not report Gazebo's measured panel positions.

Move a panel:

```bash
ros2 topic pub --once /dome/joint_command sensor_msgs/msg/JointState "{name: [pp1], position: [1.2]}"
```

## Hardware in the loop: the real dome device with the sim

With the default `servo_device:=sim`, a real device on the network drives the joints it reports, and the emulator covers the rest (see above). To have only the real device drive the panels, with no emulation at all:

```bash
ros2 launch r2_sim launch_sim.launch.py servo_device:=real
```

The real device takes `/dome/joint_command`, and its `/dome/joint_states` drives both Gazebo and `/joint_states`. Either way, the device reads its joint limits from `/robot_description`, so **the robot's own stack (`launch_robot.launch.py`) must not be running**: there must be only one `robot_state_publisher` and one source of `/joint_states`.

There are two ways to put the device on the laptop's graph:

1. **Through the robot's router (no device change).** On the robot, start only the Zenoh router (`ros2 launch r2_bringup launch_zenoh_router.launch.py`). The laptop's router already connects to it through `R2_ZENOH_CONNECT`, and the device stays pointed at the robot.
2. **Directly to the laptop's router.** Point the device's Zenoh endpoint at the laptop (`tcp/<laptop>:7447`) in its `ros` config. The robot can then be off. Point the device back at the robot afterwards.

Check that the device is present with `ros2 node list`, which should show `/r2_dome`.

## Personality

`launch_sim.launch.py` includes r2_bringup's `personality.launch.py` (`personality:=false` leaves it out) with `dome_mock:=true`: the dome mock of `r2_mocks` runs as the dome device and the expression arbiters (`/led_arbiter`, `/magic_arbiter`; claim services `/expression/claim/led` and `/expression/claim/magic`) talk to it. `ros2 run r2_expression express status` shows the channels; see the `r2_expression` README for the `express` CLI. The mock's name is the `dome_mock_name` argument (default `r2_dome`, also on `sim_foxglove.launch.py`). When a real dome device is on the same graph (the workbench), launch the sim with `dome_mock_name:=dome_mock` so the mock does not collide with `/r2_dome`; on a machine without a dome the default keeps the device's real name so everything else is identical.

## Debugging and logs

- Use `ros2 topic list` and `ros2 node info` to inspect running topics and nodes.
- `ros2 control list_controllers` shows whether `joint_broad`, `dome_state_broadcaster`, `diff_cont`, `dome_controller`, `ankle_pos` and `servo_pos` are active.
- Simulator logs are printed to the terminal where the simulation is launched.
- If controllers fail to start, examine the controller manager logs and the parameter YAML files in `config/`.

## Contributing

If you plan to add or modify simulation assets:

- Keep sim-only description changes in `urdf/gz_ros2_control.xacro`; the robot itself is described in `r2_description`.
- Add or update launch files in `launch/` and document their purpose here.
- Add parameter files under `config/` and include example usage in the corresponding launch file.

Run `colcon build --symlink-install` before submitting changes.

## License and contact

This folder follows the same license as the parent repository. If in doubt, check the workspace root for licensing information.

For questions about the simulation package, contact the project maintainers or open an issue in the main project repository.
