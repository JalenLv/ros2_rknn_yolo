# Repository Operating Guide

This file applies to the entire `ros2_rknn_yolo` Git repository and all packages
under it.

## Repository Map

- `librknn_yolo` provides the reusable RKNN inference library and task backends.
- `rknn_yolo` provides the ROS 2 node, launch files, configuration, and
  visualization support.
- `yolo_msgs` provides shared detection message definitions.
- Binary libraries and model artifacts covered by `.gitattributes` must remain
  Git LFS objects.

## Working and Commit Rules

- Preserve existing user changes and avoid destructive Git operations.
- A diagnose or review request authorizes inspection and reporting, not
  implementation.
- When asked to commit, treat the working tree and index as frozen.
- During commit preparation, only inspect, stage the already-approved
  candidate, and run checks that do not mutate source.
- If verification exposes an unexpected result, stop immediately. Report the
  branch, status, evidence, and any partial effects, then ask the user what to
  do next.
- Never fix, format, clean up, revert, or add files during a commit workflow
  without explicit authorization.
- Never push unless explicitly requested.
