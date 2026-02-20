Moveit2 pick and place Project

ROS2 + Gazebo + MoveIt2 Pick-and-Place Pipeline
Deterministic Baseline → Vision-Based Perception → AI-Powered Upgrade

Overview

Embodied Manipulation is a robotics project that implements a full pick-and-place manipulation pipeline using:

ROS2 Jazzy

Gazebo Harmonic simulation

MoveIt2 motion planning

ros2_control for joint and gripper control

OpenCV AprilTag detection for vision-based pose estimation

The goal of this project is to build a clean, modular manipulation system that:

Spawns a robot arm and scene in Gazebo

Plans collision-free trajectories using MoveIt2

Detects objects using computer vision (AprilTag)

Computes 6D object pose from camera images

Executes grasp, lift, and placement motions using detected coordinates

Why This Project

This project demonstrates core industrial robotics skills:

Motion planning and collision avoidance (MoveIt2)

Robot control integration (ros2_control, gz_ros2_control)

Computer vision and pose estimation (OpenCV, solvePnP)

Simulation setup and physics validation (Gazebo Harmonic)

TF transforms and coordinate reasoning

Clean ROS2 package architecture

Orchestrated manipulation behavior

It complements mobile robot autonomy projects by showcasing manipulation, perception, and planning capabilities.

Phase 1 — Deterministic Pick-and-Place (Baseline)

In the baseline version:

The scene contains a fixed table and objects

Object poses are known (hardcoded)

Pick and place poses are predefined

The robot executes the full sequence autonomously

Pipeline:

Move to home pose → Pre-grasp → Grasp → Close gripper → Lift → Place → Open gripper → Return home

Phase 2A — AprilTag Vision-Based Pick-and-Place (Current)

Instead of hardcoded grasp coordinates, the robot sees the object through an overhead
RGB-D camera, detects an AprilTag on the box, computes its 3D world-frame pose via
classical geometry (solvePnP), and sends that pose to MoveIt2 for grasp planning.

Architecture:

  Gazebo (RGB-D camera) → ros_gz_bridge → pose_estimator → /detected_pose → pick_place_node → MoveIt2

Components:

1. RGB-D camera in Gazebo (pick_place.sdf)
   - Fixed overhead camera at (0.6, 0, 1.0) looking straight down at the table
   - Publishes RGB image, depth image, and camera_info via gz-sim-sensors-system
   - ros_gz_bridge forwards Gazebo topics to ROS2 (including /clock for sim time sync)

2. AprilTag on the red box (custom Gazebo model: red_box_apriltag)
   - Tag36h11 family, ID 0, known physical size (0.03m)
   - Applied as PBR albedo texture on top face of the 0.04m red cube
   - Model uses Gazebo model:// URI for texture resolution

3. Pose estimator node (pick_place_control/pose_estimator.py)
   - Subscribes to /rgbd_camera/image (RGB) and /rgbd_camera/camera_info
   - Detects AprilTag using OpenCV's cv2.aruco with DICT_APRILTAG_36h11
   - Computes 6D pose via cv2.solvePnP with known tag size + camera intrinsics
   - Transforms pose from camera optical frame to world frame using precomputed transform
   - Publishes result on /detected_pose (PoseStamped) — accurate to ~1mm

4. Modified pick-and-place node (pick_place_control/pick_place_node.py)
   - Subscribes to /detected_pose instead of using hardcoded coordinates
   - Waits for first detection before starting the sequence
   - Computes pre-grasp/grasp/lift poses relative to detected tag position
   - Place location remains fixed (not vision-based)
   - Gripper control, attach/detach, and planning scene remain unchanged

5. Sim time synchronization
   - /clock topic bridged from Gazebo to ROS2
   - All nodes use use_sim_time: true for proper timestamp coordination
   - move_group.launch.py supports use_gazebo flag to skip mock controllers

Phase 2B — YOLO + Depth Centroid (Planned)

Replace AprilTags with YOLO-based object detection
Use depth image to compute 3D centroid of detected objects
Handle arbitrary objects without fiducial markers
Technologies: YOLOv8, depth processing, ROS2 image pipeline

Phase 2C — Open-Vocabulary Language-Guided Grasping (Planned)

Accept natural language commands: "Pick up the red box"
Use a vision-language model (OWL-ViT / GroundingDINO) for open-vocabulary detection
Parse text commands to identify target object and placement
Technologies: VLM, text-to-task parsing, semantic scene reasoning

The manipulation execution pipeline (MoveIt2) remains unchanged across all phases.

Repository Structure

moveit2_pick_place/
│
├── ros2_ws/
│   ├── src/
│   │   ├── arm_bringup/          # URDF, Gazebo world, controllers, launch, models
│   │   │   ├── launch/
│   │   │   │   └── gazebo.launch.py      # Gazebo + controllers + camera bridge
│   │   │   ├── worlds/
│   │   │   │   └── pick_place.sdf        # Table, red box, RGB-D camera
│   │   │   └── models/
│   │   │       └── red_box_apriltag/     # Custom model with AprilTag texture
│   │   │
│   │   ├── arm_moveit_config/    # MoveIt2 config (SRDF, OMPL, KDL, controllers)
│   │   │   └── launch/
│   │   │       └── move_group.launch.py  # MoveIt2 + RViz (supports use_gazebo flag)
│   │   │
│   │   └── pick_place_control/   # Python nodes
│   │       ├── pick_place_control/
│   │       │   ├── pick_place_node.py    # Orchestrator (vision-based)
│   │       │   ├── pose_estimator.py     # AprilTag detection + PnP
│   │       │   └── planning_scene_loader.py
│   │       └── launch/
│   │           └── pick_place_demo.launch.py  # Single-command demo
│   │
│   ├── build/
│   ├── install/
│   └── log/
│
└── README.md

How to Run

Prerequisites:

- ROS2 Jazzy
- Gazebo Harmonic
- MoveIt2
- Python packages: opencv-python, scipy, numpy

Build:

cd ros2_ws
colcon build
source install/setup.bash

Option A: Vision-Based Pick-and-Place with Gazebo (Phase 2A)

Terminal 1 — Gazebo simulation:

ros2 launch arm_bringup gazebo.launch.py

This starts Gazebo with the table, red box (with AprilTag), RGB-D camera, and robot.
Controllers and camera bridge (including /clock) are spawned automatically.

Terminal 2 — MoveIt2 + vision + pick-and-place:

source install/setup.bash
ros2 launch pick_place_control pick_place_demo.launch.py

This launches move_group, RViz, the pose estimator, and the pick-and-place node.
The sequence:
1. Pose estimator detects the AprilTag and publishes the box position
2. Pick-and-place node receives the detected pose
3. Robot moves: home → pre-grasp → grasp → close → lift → place → open → retreat → home

Option B: Deterministic Pick-and-Place with Mock Controllers (Phase 1)

ros2 launch arm_moveit_config move_group.launch.py

Then in a separate terminal:

ros2 run pick_place_control pick_place_node

Or use the single-command demo (mock controllers):

ros2 launch pick_place_control pick_place_demo.launch.py

Option C: Manual Exploration

Display robot in RViz (no simulation):

ros2 launch arm_bringup display.launch.py

Launch Gazebo only:

ros2 launch arm_bringup gazebo.launch.py

Launch MoveIt2 with mock controllers:

ros2 launch arm_moveit_config move_group.launch.py

In RViz: drag the goal pose marker, click Plan, then Execute.

Milestones

Phase 1 (v0.1–v0.7):
  ✓ Gazebo world loads with robot and objects
  ✓ Controllers start correctly
  ✓ MoveIt2 plans valid trajectories
  ✓ Pick-and-place executes successfully with mock controllers
  ✓ Demo mode runs with single command

Phase 2A (v0.8–v1.0):
  ✓ RGB-D camera streams images from Gazebo to ROS2
  ✓ AprilTag detected on red box with correct world-frame pose
  ✓ Pick-and-place uses vision-detected coordinates
  ✓ Sim time synchronization (clock bridge + use_sim_time)
  ✓ End-to-end Gazebo pick-and-place works

Technical Focus

This project emphasizes:

Clean separation of perception, planning, and execution

Proper use of TF transforms and coordinate frame reasoning

Classical computer vision (AprilTag detection, PnP pose estimation)

Collision-aware trajectory generation

Sim time synchronization between Gazebo and ROS2

Reusable and modular ROS2 package design

The architecture is intentionally structured to support AI perception modules
without rewriting the manipulation pipeline.

Long-Term Vision

Embodied Manipulation is intended to evolve into:

Language-guided manipulation

Vision-language grounding

Semantic scene reasoning

Learning-based grasp selection

The long-term goal is to bridge modern AI systems with classical robotics manipulation.

Author

Saman Aboutorab
AI Systems Architect → Robotics & Embodied Intelligence
