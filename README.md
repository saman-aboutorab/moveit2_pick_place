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

Planned Phase 2 — Embodied AI Upgrade

Future version will support commands like:

“Pick up the red mug and place it in the blue bin.”

Enhancements will include:

Open-vocabulary vision (CLIP / VLM)

Object segmentation

Depth-based 3D pose estimation

Text-to-task parsing

Replacing fixed grasp poses with perception-driven estimation

The manipulation execution pipeline (MoveIt2) will remain unchanged.

Repository Structure
embodied_manipulation/
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

If you’d like, I can now:

Add a small architecture diagram version for recruiters

Write a short 3-line “resume description” you can use later

Or refine this to match your exact ROS2 + robot choice (Panda vs UR5e)
