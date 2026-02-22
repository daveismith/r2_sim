# r2_sim — Simulation components for the R2 robot

This repository folder contains the simulation-only portions of the larger R2 ROS 2 project. It provides launch files, robot descriptions, world files, and any simulation-specific nodes or configuration needed to run the R2 robot in a simulator (Gazebo, Ignition, or another supported backend).

## Contents

- `launch/` — Launch files to start the simulator, robot state publishers, controllers, and related tooling.
- `worlds/` or `world/` — Example simulation worlds and environments.
- `models/` or `urdf/` — Robot models, meshes and URDF/xacro descriptions used in simulation.
- `config/` — Simulation configuration (controller parameters, sensor configs, etc.).

Not every subfolder is required; see the package layout in this workspace for actual structure.

## Requirements

- A working ROS 2 installation that matches this workspace (source the appropriate `setup.bash`).
- `colcon` for building the workspace.
- Simulator runtime (e.g., Gazebo or Ignition) if using physics-based simulation.

## Build & setup

From the workspace root (one level above `src/`):

```bash
# Build only the simulation package (recommended while developing)
colcon build --packages-select r2_sim

# Source the overlay after build
source install/setup.bash
```

If you prefer to build the entire workspace, run `colcon build` without `--packages-select`.

## Running the simulation

Start the simulation using a launch file from this package. Replace `<launch_file>` with an available launch file name from the `launch/` directory:

```bash
# example (replace with an actual file). From workspace root after sourcing:
ros2 launch r2_sim <launch_file>.launch.py
```

Check the `launch/` directory to see what entry points are available. Launch files may bring up the simulator, spawn the robot, start controllers, and enable visualization.

## Debugging and logs

- Use `ros2 topic list` and `ros2 node info` to inspect running topics and nodes.
- Simulator logs (Gazebo/Ignition) are typically printed to the terminal where the simulator is launched.
- If controllers fail to start, examine controller manager logs and parameter YAML files in `config/`.

## Contributing

If you plan to add or modify simulation assets:

- Keep models and mesh assets organized under `models/` or `urdf/`.
- Add or update launch files in `launch/` and document their purpose.
- Add parameter files under `config/` and include example usage in the corresponding launch file.

Please follow the repository's coding and contribution guidelines (if present) and run `colcon build` before submitting changes.

## License and contact

This folder follows the same license as the parent repository. If in doubt, check the workspace root for licensing information.

For questions about the simulation package, contact the project maintainers or open an issue in the main project repository.
