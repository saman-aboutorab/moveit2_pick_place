Moveit2 pick and place Project

ROS2 + Gazebo + MoveIt2 Pick-and-Place Pipeline
Deterministic Baseline → AI-Powered Upgrade

Overview

Embodied Manipulation is a robotics project that implements a full pick-and-place manipulation pipeline using:

ROS2

Gazebo simulation

MoveIt2 motion planning

ros2_control for joint and gripper control

The goal of this project is to build a clean, modular manipulation system that:

Spawns a robot arm and scene in Gazebo

Plans collision-free trajectories using MoveIt2

Executes grasp, lift, and placement motions

Provides a single command to run the entire pick-and-place sequence

This repository contains the deterministic baseline version.
A future phase will upgrade the system with open-vocabulary vision and embodied AI.

Why This Project

This project demonstrates core industrial robotics skills:

Motion planning and collision avoidance (MoveIt2)

Robot control integration (ros2_control)

Simulation setup and physics validation (Gazebo)

TF transforms and coordinate reasoning

Clean ROS2 package architecture

Orchestrated manipulation behavior (state machine or behavior tree)

It complements mobile robot autonomy projects by showcasing manipulation and planning capabilities.

Phase 1 — Deterministic Pick-and-Place (Baseline)

In the baseline version:

The scene contains a fixed table and objects

Object poses are known

Pick and place poses are predefined

The robot executes the full sequence autonomously

Pipeline:

Move to home pose

Plan to pre-grasp pose

Approach grasp pose

Close gripper and attach object

Lift object

Move to place pose

Open gripper and detach object

Return to home

The goal is reliability and clean architecture before introducing AI perception.

Phase 2 — Vision-Based Perception Upgrade

Phase 2A: AprilTag + RGB-D Pose Estimation

Add a simulated RGB-D camera to the Gazebo scene
Attach AprilTags to target objects
Detect tags using OpenCV and estimate 6D pose via PnP
Feed detected pose to MoveIt2 — replacing hardcoded grasp coordinates
Technologies: OpenCV, cv_bridge, AprilTag, Gazebo depth camera plugin

Phase 2B: YOLO + Depth Centroid

Replace AprilTags with YOLO-based object detection
Use depth image to compute 3D centroid of detected objects
Handle arbitrary objects without fiducial markers
Technologies: YOLOv8, depth processing, ROS2 image pipeline

Phase 2C: Open-Vocabulary Language-Guided Grasping

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
│   │   ├── arm_bringup/
│   │   ├── arm_moveit_config/
│   │   └── pick_place_control/
│   │
│   ├── build/
│   ├── install/
│   └── log/
│
└── README.md
System Architecture (Baseline)

High-level flow:

Gazebo → ros2_control → MoveIt2 → PickPlace Orchestrator → Trajectory Execution

Core components:

Simulation world (table + objects)

Robot arm + gripper

MoveIt2 planning pipeline

Pick-and-place control node

Planning scene object management

How to Run

Build:

cd ros2_ws
colcon build
source install/setup.bash

Display robot in RViz (no simulation):

ros2 launch arm_bringup display.launch.py

In RViz: Add > RobotModel, set Fixed Frame to panda_link0.
Use the Joint State Publisher GUI sliders to move joints.

Launch Gazebo simulation:

ros2 launch arm_bringup gazebo.launch.py

This starts Gazebo with the table, objects, and robot.
Controllers (arm + gripper + joint state broadcaster) are spawned automatically.

Launch MoveIt2 planning (without Gazebo, mock controllers):

ros2 launch arm_moveit_config move_group.launch.py

This starts move_group, RViz with MotionPlanning panel, and mock controllers.
In RViz: drag the goal pose marker, click Plan, then Execute.

Load planning scene collision objects (in a separate terminal while move_group is running):

ros2 run pick_place_control planning_scene_loader

This adds the table and objects as collision objects in MoveIt2's planning scene.
The planner will now generate trajectories that avoid these objects.

Run the pick-and-place sequence (in a separate terminal while move_group is running):

ros2 run pick_place_control pick_place_node

This executes the full autonomous pick-and-place pipeline:
1. Moves to home position
2. Loads collision objects into the planning scene
3. Opens gripper
4. Moves to pre-grasp pose above the target object
5. Descends to grasp pose
6. Closes gripper and attaches object in planning scene
7. Lifts object
8. Moves to place location
9. Opens gripper and detaches object
10. Retreats upward and returns to home

Run the full demo with a single command:

ros2 launch pick_place_control pick_place_demo.launch.py

This launches everything in one go: move_group, RViz, mock controllers, and the
pick-and-place node. The pick-and-place sequence starts automatically after a delay.

Milestones

 Gazebo world loads with robot and objects

 Controllers start correctly

 MoveIt2 plans valid trajectories

 Pick-and-place executes successfully

 Demo mode runs with single command

 Success rate ≥ 80%

Technical Focus

This project emphasizes:

Clean separation of perception, planning, and execution

Proper use of TF transforms

Collision-aware trajectory generation

Reusable and modular ROS2 package design

The architecture is intentionally structured to support AI perception modules without rewriting the manipulation pipeline.

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
