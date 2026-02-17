# Implementation Steps — MoveIt2 Pick-and-Place Project

This file is the master reference for all implementation steps.
Each step should be completed and verified before moving to the next.

---

## Phase 1 — MVP (Deterministic Pick-and-Place)

### Step 1: Project & Build Infrastructure
- Create the ROS2 workspace structure (`ros2_ws/src/` with 3 packages):
  - `arm_bringup` — launch Gazebo, spawn robot + scene, start controllers
  - `arm_moveit_config` — MoveIt2 configuration
  - `pick_place_control` — orchestrator node
- Set up `package.xml` and `CMakeLists.txt` for each package
- **Verify:** `colcon build` succeeds with no errors

### Step 2: Robot Description (URDF/XACRO)
- Select robot arm: **Panda** (best out-of-the-box MoveIt2 support)
- Bring in the robot description (xacro) with the Panda parallel gripper
- Add `ros2_control` hardware interface tags (Gazebo plugin)
- Add Gazebo sensor plugins if needed for the gripper
- **Verify:** `ros2 launch ... display.launch.py` shows the robot correctly in RViz

### Step 3: Gazebo World & Scene
- Create a simple `.sdf` or `.world` file: ground plane + table + 1–3 primitive objects (boxes/cylinders)
- Write a launch file that starts Gazebo and spawns the robot on/near the table
- Object poses are fixed and loaded from a YAML config
- **Verify:** robot and scene appear correctly in Gazebo

### Step 4: ros2_control Controllers
- Configure controller YAML:
  - `joint_trajectory_controller` for the arm
  - Position controller for the gripper
- Launch `controller_manager` and spawn controllers
- **Verify:** joints can be moved via `ros2 topic pub` or `ros2 control` CLI commands

### Step 5: MoveIt2 Configuration
- Create the `arm_moveit_config` package:
  - SRDF (planning groups: arm, gripper)
  - Kinematics solver config (KDL or similar)
  - OMPL planning pipeline config
  - Joint limits YAML
  - Controller interface config for MoveIt2
- Launch `move_group` node
- **Verify:** RViz MotionPlanning plugin can plan and execute to random targets

### Step 6: Planning Scene Integration
- Add collision objects (table, objects) to the MoveIt2 planning scene
- Load object dimensions and poses from YAML config
- **Verify:** collision objects are visible in RViz; motion plans avoid them

### Step 7: Pick-and-Place Orchestrator Node
- Write the `pick_place_control` node (Python or C++) implementing the full sequence:
  1. Move to home pose
  2. Plan to pre-grasp pose
  3. Cartesian or planned approach to grasp pose
  4. Close gripper + attach object in planning scene
  5. Lift object
  6. Move to pre-place pose / place pose
  7. Open gripper + detach object
  8. Return to home pose
- Expose a ROS2 **service**: `run_pick_and_place(pick_pose, place_pose, object_id)`
- Provide a **demo mode** with default poses that runs with one command

### Step 8: Top-Level Bringup Launch
- Create a single launch file that chains everything:
  Gazebo → spawn robot → controllers → MoveIt2 → orchestrator
- One command to run the full demo:
  ```
  ros2 launch arm_bringup pick_and_place_demo.launch.py
  ```

### Step 9: Testing & Polish
- Run the full demo end-to-end; debug failure cases (planning failures, gripper timing, collisions)
- Add clear logging and error handling throughout:
  - Planning failed
  - Grasp failed
  - Controller not available
- Update `README.md` with final build/run instructions and expected output
- Target: **≥ 80% success rate** on repeated runs

---

## Phase 2 — AI Upgrade (Perception-Driven Manipulation)

> **Prerequisite:** Phase 1 must be fully working before starting Phase 2.

### Step 10: RGB-D Camera Setup
- Add a depth camera to the Gazebo scene (e.g., Gazebo `rgbd_camera` sensor)
  - Mounting options: overhead (fixed) or wrist-mounted
- Publish on standard ROS2 topics: `image_raw`, `depth`, `camera_info`
- **Verify:** `ros2 topic echo` / RViz image display shows the camera feed with objects visible

### Step 11: Open-Vocabulary Object Detection
- Integrate a vision model as a ROS2 node (e.g., **GroundingDINO**, **OWL-ViT**, or **YOLO-World**)
- Input: RGB image + text query (e.g., `"red mug"`)
- Output: 2D bounding box + label + confidence score
- **Verify:** given a text prompt, the node draws correct bounding boxes on the camera feed

### Step 12: 3D Pose Estimation
- Use the 2D detection + depth image + camera intrinsics to project to a **3D pick pose** in the robot's base frame
- TF lookup: `camera_frame` → `base_frame`
- Output: `geometry_msgs/PoseStamped` of the target object
- **Verify:** publish a marker in RViz at the estimated pose; confirm it aligns with the object in Gazebo

### Step 13: Text-Command Interface
- Create a command node that accepts natural language input (via service call or CLI):
  - e.g., `"Pick up the red box and place it on the left side of the table"`
- Parse the command to extract:
  - **Object query** (what to pick)
  - **Place location** (where to put it)
- Can start simple (keyword extraction) or use an LLM for parsing
- Feed the detected 3D pose into the **existing** `run_pick_and_place` service
- **Key:** no changes to the MoveIt2 orchestrator pipeline

### Step 14: End-to-End AI Integration & Testing
- Single workflow: type/speak a sentence → vision detects → pose estimated → pick-and-place executes
- Handle failure cases:
  - Object not found in scene
  - Pose is unreachable
  - Grasp fails
- Test with multiple objects, different prompts, and varying object positions
- **Verify:** reliable end-to-end execution with natural language commands

---

## Architecture Summary

```
Phase 1 (MVP):
  Fixed YAML poses → PickPlace Orchestrator → MoveIt2 → ros2_control → Gazebo

Phase 2 (AI Upgrade):
  Text Command → Vision Node → 3D Pose Estimation → PickPlace Orchestrator → MoveIt2 → ros2_control → Gazebo
```

**Key design principle:** The AI perception layer (Phase 2) only replaces the *"where to pick"* input. The MoveIt2 orchestrator from Phase 1 stays untouched.

---

## Package Responsibilities

| Package | Role |
|---------|------|
| `arm_bringup` | Launch Gazebo, spawn robot + scene, start controllers |
| `arm_moveit_config` | MoveIt2 SRDF, kinematics, OMPL, joint limits, controller config |
| `pick_place_control` | Orchestrator node, service interface, demo mode |
