Moveit2 pick and place Project

ROS2 + Gazebo + MoveIt2 Pick-and-Place Pipeline
Deterministic Baseline → Vision-Based Perception → AI-Powered Upgrade

Overview

Embodied Manipulation is a robotics project that implements a full pick-and-place manipulation pipeline using:

ROS2 Jazzy

Gazebo Harmonic simulation

MoveIt2 motion planning

ros2_control for joint and gripper control

Multiple perception backends (AprilTag, YOLO, VLM) — switchable via launch argument

The goal of this project is to build a clean, modular manipulation system that:

Spawns a robot arm and scene in Gazebo

Plans collision-free trajectories using MoveIt2

Detects objects using interchangeable computer vision backends

Computes 3D object pose from camera images

Executes grasp, lift, and placement motions using detected coordinates

Why This Project

This project demonstrates core industrial robotics skills:

Motion planning and collision avoidance (MoveIt2)

Robot control integration (ros2_control, gz_ros2_control)

Computer vision and pose estimation (OpenCV, YOLO, solvePnP)

Simulation setup and physics validation (Gazebo Harmonic)

TF transforms and coordinate reasoning

Clean ROS2 package architecture

Orchestrated manipulation behavior

It complements mobile robot autonomy projects by showcasing manipulation, perception, and planning capabilities.

Architecture

The system is designed with a clean separation between perception and manipulation.
All detector backends publish to the same `/detected_pose` topic, so the pick-and-place
node works identically regardless of which detector is running:

  Gazebo (RGB-D camera)
       │
       ├── /rgbd_camera/image
       ├── /rgbd_camera/depth_image
       └── /rgbd_camera/camera_info
              │
              ▼
  ┌─────────────────────────┐
  │   Detector Node         │  ← swappable via detector:= launch arg
  │   (apriltag / yolo / vlm)│
  └────────────┬────────────┘
               │ /detected_pose (PoseStamped)
               ▼
  ┌─────────────────────────┐
  │   pick_place_node       │  ← unchanged across all phases
  │   (MoveIt2 orchestrator)│
  └─────────────────────────┘

Switching detectors:

  ros2 launch pick_place_control pick_place_demo.launch.py detector:=apriltag  # Phase 2A
  ros2 launch pick_place_control pick_place_demo.launch.py detector:=yolo      # Phase 2B
  ros2 launch pick_place_control pick_place_demo.launch.py detector:=vlm       # Phase 2C

Phase 1 — Deterministic Pick-and-Place (Baseline)

In the baseline version:

The scene contains a fixed table and objects

Object poses are known (hardcoded)

Pick and place poses are predefined

The robot executes the full sequence autonomously

Pipeline:

Move to home pose → Pre-grasp → Grasp → Close gripper → Lift → Place → Open gripper → Return home

Phase 1 is preserved in git history (tags v0.1–v0.7).

Phase 2A — AprilTag Vision-Based Pick-and-Place

Instead of hardcoded grasp coordinates, the robot sees the object through an overhead
RGB-D camera, detects an AprilTag on the box, computes its 3D world-frame pose via
classical geometry (solvePnP), and sends that pose to MoveIt2 for grasp planning.

Components:

1. RGB-D camera in Gazebo (pick_place.sdf)
   - Fixed overhead camera at (0.6, 0, 1.0) looking straight down at the table
   - Publishes RGB image, depth image, and camera_info via gz-sim-sensors-system
   - ros_gz_bridge forwards Gazebo topics to ROS2 (including /clock for sim time sync)

2. AprilTag on the red box (custom Gazebo model: red_box_apriltag)
   - Tag36h11 family, ID 0, known physical size (0.03m)
   - Applied as PBR albedo texture on top face of the 0.04m red cube
   - Model uses Gazebo model:// URI for texture resolution

3. Pose estimator node (pose_estimator.py)
   - Subscribes to /rgbd_camera/image (RGB) and /rgbd_camera/camera_info
   - Detects AprilTag using OpenCV's cv2.aruco with DICT_APRILTAG_36h11
   - Computes 6D pose via cv2.solvePnP with known tag size + camera intrinsics
   - Transforms pose from camera optical frame to world frame
   - Publishes on /detected_pose (PoseStamped) — accurate to ~1mm

Detection method: Classical geometry (no ML). Requires a fiducial marker on the object.

Phase 2B — YOLO + Depth Centroid (In Progress)

Replace the AprilTag detector with YOLOv8 neural network object detection.
The robot detects the red box by appearance (no marker needed), then uses the
depth image to compute the object's 3D position.

Components:

1. YOLO detector node (yolo_detector.py) — replaces pose_estimator.py
   - Subscribes to /rgbd_camera/image (RGB) and /rgbd_camera/depth_image
   - Runs YOLOv8 inference on RGB frame to detect objects by class
   - Computes bounding box center pixel (u, v)
   - Reads depth value d at that pixel from the depth image
   - Projects (u, v, d) to 3D world coordinates using camera intrinsics + extrinsics:
       x_cam = (u - cx) * d / fx
       y_cam = (v - cy) * d / fy
       z_cam = d
   - Transforms from camera optical frame to world frame (same transform as 2A)
   - Publishes on /detected_pose (PoseStamped) — same topic, same message type

2. No changes needed to:
   - pick_place_node.py (subscribes to /detected_pose — works with any detector)
   - gazebo.launch.py (camera and bridge already in place)
   - Gazebo world (red box still present, AprilTag texture optional)

Implementation steps:
  Step 1: Install ultralytics (YOLOv8) and verify inference on a test image
  Step 2: Create yolo_detector.py node with RGB + depth subscriptions
  Step 3: Add depth-to-world projection using camera intrinsics + extrinsics
  Step 4: Add entry point and update launch file with detector:= argument
  Step 5: Test end-to-end

Detection method: Neural network (YOLOv8). No markers needed — detects by appearance.
Accuracy: ~1-2cm (position only from depth centroid, no rotation estimation).

Dependencies: ultralytics (pip install ultralytics)

Phase 2C — Open-Vocabulary Language-Guided Grasping (Planned)

Accept natural language commands: "Pick up the red box"
Use a vision-language model (OWL-ViT / GroundingDINO) for open-vocabulary detection
Parse text commands to identify target object and placement
Technologies: VLM, text-to-task parsing, semantic scene reasoning

Detection method: Vision-language model. Detects objects from text descriptions.

The manipulation execution pipeline (MoveIt2) remains unchanged across all phases.

Branching & Git Strategy

Each phase is developed on a feature branch, then merged into main when complete.
All detector backends coexist as separate files on main — no conflicts between phases.

  main ─────────── merge 2A ─── merge 2B ─── merge 2C ──→
    \              /              /              /
     feature/phase-2a-apriltag  /              /
                    \          /              /
                     feature/phase-2b-yolo  /
                                  \        /
                                   feature/phase-2c-vlm

Files on main after all phases:

  pick_place_control/
    ├── pose_estimator.py     # Phase 2A — AprilTag + solvePnP
    ├── yolo_detector.py      # Phase 2B — YOLO + depth centroid
    ├── vlm_detector.py       # Phase 2C — Vision-language model
    └── pick_place_node.py    # Shared orchestrator (unchanged)

The demo launch file uses a `detector` argument to select which backend to run.
All detectors publish to `/detected_pose` so the orchestrator works with any of them.

Tags:
  v0.1–v0.7  Phase 1 (deterministic baseline)
  v0.8–v1.0  Phase 2A (AprilTag)
  v1.1+      Phase 2B (YOLO)

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
│   │       │   ├── pose_estimator.py     # Phase 2A: AprilTag detector
│   │       │   ├── yolo_detector.py      # Phase 2B: YOLO detector
│   │       │   └── planning_scene_loader.py
│   │       └── launch/
│   │           └── pick_place_demo.launch.py  # Single-command demo (detector:= arg)
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
- For Phase 2B: ultralytics (pip install ultralytics)

Build:

cd ros2_ws
colcon build
source install/setup.bash

Vision-Based Pick-and-Place with Gazebo

Terminal 1 — Gazebo simulation:

ros2 launch arm_bringup gazebo.launch.py

This starts Gazebo with the table, red box, RGB-D camera, and robot.
Controllers and camera bridge (including /clock) are spawned automatically.

Terminal 2 — MoveIt2 + detector + pick-and-place:

source install/setup.bash

# AprilTag detector (Phase 2A):
ros2 launch pick_place_control pick_place_demo.launch.py detector:=apriltag

# YOLO detector (Phase 2B):
ros2 launch pick_place_control pick_place_demo.launch.py detector:=yolo

The sequence is the same regardless of detector:
1. Detector node identifies the object and publishes its world-frame position
2. Pick-and-place node receives the detected pose
3. Robot moves: home → pre-grasp → grasp → close → lift → place → open → retreat → home

Manual Exploration

Display robot in RViz (no simulation):

ros2 launch arm_bringup display.launch.py

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

Phase 2B (v1.1+):
  ○ YOLOv8 detects red box without fiducial markers
  ○ Depth centroid provides 3D object position
  ○ Detector switchable via launch argument
  ○ End-to-end Gazebo pick-and-place works

Technical Focus

This project emphasizes:

Clean separation of perception, planning, and execution

Proper use of TF transforms and coordinate frame reasoning

Classical computer vision (AprilTag detection, PnP pose estimation)

Deep learning vision (YOLO object detection)

Collision-aware trajectory generation

Sim time synchronization between Gazebo and ROS2

Reusable and modular ROS2 package design

The architecture is intentionally structured so that swapping the perception
backend requires only a new detector node — the manipulation pipeline is untouched.

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
