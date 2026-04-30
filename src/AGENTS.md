# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 workspace `src` directory with several packages. Core C++ packages follow the usual `include/<package>/` and `src/` layout: `agv_hardware`, `agv_protocol`, `agv_bridge`, `robot_move`, and `camera`. Runtime orchestration lives in `agv_bringup/launch` and `startup/src`. Robot assets and simulation/configuration files are in `agv_description/{urdf,launch,config}` and `agv_moveit_config/{config,launch}`. Path planning is a Python package under `pathplannernode/src/pathplanner`, with planner modules in `pathplanner/`, ROS node wrappers in `pathplanner_ros/`, service definitions in `srv/`, and parameters in `pathplannernode/src/config`.

## Build, Test, and Development Commands

Run commands from the workspace root, one directory above this `src` folder.

- `colcon build --symlink-install`: build all ROS 2 packages.
- `colcon build --symlink-install --packages-select agv_hardware`: rebuild one package while iterating.
- `source install/setup.bash`: overlay the built workspace in the current shell.
- `colcon test --event-handlers console_direct+`: run package tests and lint checks.
- `colcon test-result --verbose`: inspect failures after a test run.
- `ros2 launch agv_bringup agv.launch.xml`: start the main AGV bringup launch file.
- `ros2 launch pathplanner ros2_pathplanner_launch.py`: start the path planner node after building and sourcing.

## Coding Style & Naming Conventions

Use C++17-style ROS 2 conventions already present in the code: headers in `include/<package>/`, implementations in `src/`, snake_case file names, and package-scoped namespaces. Keep CMake target names descriptive and package-local. Python code should use 4-space indentation, snake_case modules/functions, and ROS entry points declared in `setup.py`. Keep launch/config names explicit, for example `ros2_controllers.yaml` or `ros2_pathplanner_launch.py`.

## Testing Guidelines

Most CMake packages declare `ament_lint_auto` dependencies; keep new packages wired into `BUILD_TESTING` where practical. The Python pathplanner package declares `pytest`, `ament_flake8`, and `ament_pep257`; add tests near the package they exercise and name Python tests `test_*.py`. Prefer `colcon test --packages-select <package>` for targeted validation before running the full workspace test suite.

## Commit & Pull Request Guidelines

Recent history uses short imperative Chinese summaries, such as `修改路径规划启动` and `添加工具文件`. Keep commit subjects concise and action-oriented; include the affected subsystem when helpful. Pull requests should describe the behavior change, list the packages touched, include launch/test commands run, and call out hardware, network, or robot-safety assumptions. Add screenshots or RViz captures when changing visualization, URDF, MoveIt, or UI-facing behavior.

## Security & Configuration Tips

Do not commit machine-specific IPs, credentials, or generated outputs unless they are intentional fixtures. Treat hardware SDK libraries under `agv_hardware/lib` and large planning outputs such as `pathplannernode/result` as managed artifacts; avoid replacing them without documenting source and platform.
