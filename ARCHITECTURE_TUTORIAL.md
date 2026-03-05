# MoveIt2 Pick-and-Place: Complete Beginner's Tutorial

---

## Table of Contents

1. [What Is This Project?](#1-what-is-this-project)
2. [Big Picture Architecture](#2-big-picture-architecture)
3. [The Three Layers of the System](#3-the-three-layers-of-the-system)
4. [Package-by-Package Breakdown](#4-package-by-package-breakdown)
5. [Every Node Explained](#5-every-node-explained)
6. [Every Topic, Message Type & Sample Data](#6-every-topic-message-type--sample-data)
7. [End-to-End Scenario: "Pick the Blue Box"](#7-end-to-end-scenario-pick-the-blue-box)
8. [Coordinate Systems](#8-coordinate-systems)
9. [How Controllers Work](#9-how-controllers-work)
10. [Vision Backends Compared](#10-vision-backends-compared)
11. [Deep Dive: YOLO → pick_place_node Function-by-Function](#11-deep-dive-yolo--pick_place_node-function-by-function)

---

## 1. What Is This Project?

This project teaches a **robot arm (Panda)** to:
1. **See** an object on a table using a camera
2. **Plan** a safe path to reach it (avoiding collisions)
3. **Pick it up** by closing its gripper
4. **Place it** at a target location
5. **Return home**

All in simulation — no real hardware needed.

```
         CAMERA (sees the table)
              │
              ▼
    ┌─────────────────────┐
    │   VISION NODE       │  "I see the blue box at position (0.6, -0.12, 0.44)"
    └─────────┬───────────┘
              │  /detected_pose
              ▼
    ┌─────────────────────┐
    │  PICK_PLACE_NODE    │  "Okay, move arm to hover, descend, grip, lift, place"
    └─────────┬───────────┘
              │  MoveIt2 planning requests
              ▼
    ┌─────────────────────┐
    │    MOVE_GROUP       │  "Here's the joint trajectory to execute safely"
    └─────────┬───────────┘
              │  joint trajectory commands
              ▼
    ┌─────────────────────┐
    │  GAZEBO (robot)     │  Arm actually moves in simulation
    └─────────────────────┘
```

---

## 2. Big Picture Architecture

This is the full system at a glance:

```
╔══════════════════════════════════════════════════════════════════════════╗
║                         GAZEBO SIMULATION                                ║
║                                                                          ║
║   ┌──────────┐   ┌──────────────┐   ┌────────────────────┐              ║
║   │  Table   │   │  Boxes/Objs  │   │   RGBD Camera      │              ║
║   │(static)  │   │  (on table)  │   │  (overhead,z=1.0m) │              ║
║   └──────────┘   └──────────────┘   └───────┬────────────┘              ║
║                                             │ publishes images           ║
║   ┌─────────────────────────────────────────┼───────────────────┐       ║
║   │         PANDA ROBOT ARM                 │                   │       ║
║   │  7 joints + gripper (2 fingers)         │                   │       ║
║   │  Controlled by ros2_control             │                   │       ║
║   └─────────────────────────────────────────┘                   │       ║
╚═════════════════════════════════════════════════════════════════╪════════╝
                                                                  │
                          ros_gz_bridge                           │
                    (Gazebo ↔ ROS2 translator)                    │
                                                                  │
╔═════════════════════════════════════════════════════════════════╪════════╗
║                          ROS2 WORLD                             │        ║
║                                                                 ▼        ║
║  /rgbd_camera/image ◄────────────────────────────────────────────────── ║
║  /rgbd_camera/depth_image ◄─────────────────────────────────────────── ║
║  /rgbd_camera/camera_info ◄─────────────────────────────────────────── ║
║                                                                          ║
║  ┌──────────────────────────────────────────────┐                        ║
║  │           VISION NODE (choose one)           │                        ║
║  │  ┌────────────────┐  ┌────────────────────┐  │                        ║
║  │  │ pose_estimator │  │  yolo_detector     │  │                        ║
║  │  │ (AprilTag 2A)  │  │  (YOLOv8 2B)      │  │                        ║
║  │  └────────────────┘  └────────────────────┘  │                        ║
║  │           ┌────────────────────┐             │                        ║
║  │           │   vlm_detector     │             │                        ║
║  │           │  (OWL-ViT 2C)     │             │                        ║
║  │           └────────────────────┘             │                        ║
║  └──────────────────┬───────────────────────────┘                        ║
║                     │ /detected_pose (PoseStamped)                       ║
║                     ▼                                                     ║
║  ┌──────────────────────────────────────────────┐                        ║
║  │           pick_place_node.py                 │                        ║
║  │  (Orchestrator - "brain" of the robot)       │                        ║
║  └──────┬──────────────────────┬────────────────┘                        ║
║         │ pymoveit2 API        │ /hand_trajectory_controller/             ║
║         │ (motion requests)    │   joint_trajectory                       ║
║         ▼                      ▼                                          ║
║  ┌─────────────────┐   ┌──────────────────────┐                          ║
║  │   MOVE_GROUP    │   │  hand_trajectory_     │                          ║
║  │  (MoveIt2 node) │   │  controller          │                          ║
║  │                 │   │  (gripper motor)      │                          ║
║  │  - OMPL planner │   └──────────────────────┘                          ║
║  │  - KDL IK solver│                                                      ║
║  │  - Collision    │                                                      ║
║  │    checking     │                                                      ║
║  └────────┬────────┘                                                      ║
║           │ /panda_arm_controller/follow_joint_trajectory                 ║
║           ▼                                                               ║
║  ┌─────────────────────────────────────────────────────────┐             ║
║  │              panda_arm_controller                        │             ║
║  │      (sends position commands to Gazebo joints)          │             ║
║  └────────────────────────────┬────────────────────────────┘             ║
╚═══════════════════════════════╪════════════════════════════════════════  ╝
                                │  ros2_control → Gazebo
                                ▼
                      Robot arm physically moves
```

---

## 3. The Three Layers of the System

Think of the system as three stacked layers:

```
┌─────────────────────────────────────────────────────────┐
│  LAYER 3: APPLICATION (What to do)                       │
│                                                          │
│  pick_place_node.py  ←  vision nodes                    │
│  "See box → pick → place"                               │
└────────────────────────┬────────────────────────────────┘
                         │ asks for plans
┌────────────────────────▼────────────────────────────────┐
│  LAYER 2: PLANNING (How to do it safely)                 │
│                                                          │
│  move_group (MoveIt2)                                   │
│  "Plan collision-free path, solve inverse kinematics"   │
└────────────────────────┬────────────────────────────────┘
                         │ sends joint commands
┌────────────────────────▼────────────────────────────────┐
│  LAYER 1: HARDWARE/SIMULATION (Physically do it)        │
│                                                          │
│  Gazebo + ros2_control + Controllers                    │
│  "Move the actual robot joints"                         │
└─────────────────────────────────────────────────────────┘
```

---

## 4. Package-by-Package Breakdown

```
ros2_ws/src/
├── arm_bringup/          ← "The Robot's Body"
│   Everything needed to LAUNCH the physical robot or simulation:
│   - URDF: robot shape and joints description
│   - Gazebo world: table, boxes, camera
│   - Controllers config: how to drive the joints
│   - Launch files: start up Gazebo
│
├── arm_moveit_config/    ← "The Robot's Brain Config"
│   Everything MoveIt2 needs to PLAN motions:
│   - SRDF: motion groups ("arm" = joints 1-7)
│   - OMPL: planner algorithm settings
│   - Kinematics (KDL): how to solve IK
│   - Joint limits: max speed/acceleration
│   - Controller interface: how MoveIt2 sends commands
│
└── pick_place_control/   ← "The Application"
    The actual pick-and-place program:
    - Vision nodes (camera → object position)
    - pick_place_node (orchestrator)
    - Scene config (collision objects YAML)
    - Main launch file
```

---

## 5. Every Node Explained

### Node 1: `pick_place_node.py` — The Brain

**What it is**: The top-level orchestrator. It sequences every step of the pick-and-place task.

**Think of it as**: A chef following a recipe step-by-step.

```
RECIPE (sequence of steps):
Step 1:  Wait for vision node to detect an object
Step 2:  Move arm to HOME position
Step 3:  Load collision table into planning scene
Step 4:  Open gripper
Step 5:  Move to PRE-GRASP (hover 15cm above box)
Step 6:  Descend to GRASP position
Step 7:  Close gripper (grab the box)
Step 8:  Attach box to arm in planning scene
Step 9:  LIFT (move up)
Step 10: Move to PLACE position
Step 11: Detach box from arm in planning scene
Step 12: Open gripper (release)
Step 13: RETREAT (move back up)
Step 14: Return HOME
```

**Key design choices**:
- Uses `pymoveit2` library (not `moveit_py`) as the Python interface to MoveIt2
- Uses `ReentrantCallbackGroup` + `MultiThreadedExecutor` (required so the node can both receive topics AND wait for motion to finish simultaneously)
- Controls gripper with direct topic publish (not an action) to avoid pymoveit2 gripper bugs

```python
# How it moves the arm (simplified):
self.moveit2.move_to_pose(
    position=[x, y, z],        # Where to move the tip
    quat_xyzw=[0, 1, 0, 0],   # Orientation (pointing down)
    cartesian=True,            # Move in straight line (not arc)
)
self.moveit2.wait_until_executed()
```

```python
# How it moves the gripper:
msg = JointTrajectory()
msg.joint_names = ['panda_finger_joint1']
point = JointTrajectoryPoint()
point.positions = [0.04]   # open = 0.04, closed = 0.0
point.time_from_start = Duration(seconds=2)
msg.points = [point]
self.gripper_pub.publish(msg)   # sends to /hand_trajectory_controller/joint_trajectory
```

**Important constants**:
```python
HOME_JOINT_POSITIONS = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
HAND_LENGTH    = 0.12   # m, panda_link8 to fingertips (pointing down)
BOX_HALF_HEIGHT = 0.02  # m
PRE_GRASP_HEIGHT = 0.15 # m, hover above grasp point
PLACE_POSITION  = [0.5, 0.3, 0.54]
RETREAT_POSITION = [0.5, 0.3, 0.69]
GRIPPER_OPEN    = [0.04]
GRIPPER_CLOSED  = [0.0]
```

---

### Node 2: `pose_estimator.py` — AprilTag Vision (Phase 2A)

**What it is**: Looks for an AprilTag (a square barcode-like marker) on the box and figures out exactly where the box is in 3D space.

**Think of it as**: Scanning a QR code to know "this box is 60cm in front and 12cm to the left."

**Pipeline step-by-step**:

```
RGB Image (640x480)
       │
       ▼
  Convert to grayscale
       │
       ▼
  OpenCV ArUco detector
  "Find all 4 corners of the AprilTag in 2D image pixels"
       │
       ▼
  solvePnP (Perspective-n-Point)
  "Given 4 known 3D tag corners and their 2D pixel positions,
   compute the camera-to-tag transformation (6D pose)"
       │
       ▼
  Transform camera frame → world frame
  (using known camera position in the world)
       │
       ▼
  Publish /detected_pose (PoseStamped)
  "box is at world position (0.6, 0.0, 0.42)"
```

**The math behind solvePnP**:
```
solvePnP solves:

    [u]   [fx  0  cx]   [R | t]   [X]
    [v] = [ 0 fy  cy] × [0 | 1] × [Y]
    [1]   [ 0  0   1]              [Z]
                                   [1]

Where:
  (u,v) = pixel coordinates of tag corner  (known from image)
  fx,fy = focal lengths                     (known from camera_info)
  cx,cy = principal point                   (known from camera_info)
  R,t   = rotation + translation            ← what we're solving for
  X,Y,Z = known 3D tag corner in tag frame  (known: tag is 3cm × 3cm)
```

---

### Node 3: `yolo_detector.py` — YOLO Vision (Phase 2B)

**What it is**: Uses a deep learning neural network (YOLOv8) to recognize the box in the image and then uses the depth camera to find its 3D position.

**Think of it as**: Object recognition ("that's a bottle!") + GPS lookup ("it's 60cm away").

**Pipeline step-by-step**:

```
RGB Image                     Depth Image
     │                              │
     ▼                              │
YOLOv8 inference                   │
"I see a 'bottle' at              │
 bounding box [x1,y1,x2,y2]"      │
     │                              │
     ▼                              │
Compute bounding box center        │
  u = (x1+x2)/2                   │
  v = (y1+y2)/2                   │
     │                              │
     ├──────────────────────────────┘
     ▼
Sample depth at pixel (u,v)
  depth = depth_image[v, u]   (e.g., 0.58 meters)
     │
     ▼
Back-project to 3D camera frame
  x_cam = (u - cx) × depth / fx
  y_cam = (v - cy) × depth / fy
  z_cam = depth
     │
     ▼
Transform camera frame → world frame
     │
     ▼
Publish /detected_pose
```

**Fallback**: If YOLO can't detect the object (confidence too low), it falls back to **HSV color masking** — finds red-colored pixels and uses their centroid.

**Parameters**:
```python
model='yolov8n.pt'          # Nano model (fastest/smallest)
target_class='bottle'       # Configurable class name
conf_threshold=0.3          # 30% confidence minimum
use_color_fallback=True     # HSV red fallback for Gazebo
```

---

### Node 4: `vlm_detector.py` — OWL-ViT Vision (Phase 2C)

**What it is**: Uses a Vision-Language Model (OWL-ViT from Google) to find objects described in **plain English** — no pre-training needed.

**Think of it as**: Telling the computer "find me the blue box" in natural language, and it does it.

**What's special**: Unlike YOLO (fixed list of classes), OWL-ViT can find **any object you describe in words**.

```
target_object = "blue box"    ← plain English!

RGB Image + text prompt "blue box"
           │
           ▼
    OWL-ViT Model
    (Vision Transformer + text encoder)
           │
           ▼
    Returns bounding boxes scored by
    similarity to "blue box"
           │
           ▼
    Same depth back-projection as YOLO
           │
           ▼
    Publish /detected_pose
```

**Important implementation detail**: This node uses `numpy.frombuffer()` instead of `cv_bridge` to convert ROS images, because the system has NumPy 2.x (installed by PyTorch/HuggingFace) which crashes the C extension in `cv_bridge`.

**Parameters**:
```python
target_object='blue box'              # Natural language prompt
model='google/owlvit-base-patch32'    # ~580 MB from HuggingFace
conf_threshold=0.05                   # Low for Gazebo geometry
use_color_fallback=True               # HSV blue fallback
```

---

### Node 5: `move_group` — The Motion Planner

**What it is**: A core MoveIt2 node that handles all motion planning. It is NOT code you wrote — it's a standard MoveIt2 component.

**What it does**:
- Solves **Inverse Kinematics** (IK): "What joint angles do I need so the arm tip ends up at position X,Y,Z?"
- Plans **collision-free paths** using OMPL (Open Motion Planning Library)
- Tracks the **planning scene** (where is the table? where is the box?)
- Executes plans by sending commands to controllers

```
Request from pick_place_node:
"Move tip to position (0.6, 0.0, 0.52), pointing down"
                    │
                    ▼
            KDL IK Solver
    "What 7 joint angles achieve this tip pose?"
    Answer: [j1=-0.12, j2=-0.91, j3=0.15, j4=-2.1, j5=0.05, j6=1.8, j7=0.85]
                    │
                    ▼
            OMPL Planner (RRTConnect)
    "Plan a path from current joint angles to target angles
     without hitting the table or any obstacle"
    Answer: [waypoint1, waypoint2, ..., waypoint_N]  ← trajectory
                    │
                    ▼
    Send trajectory to panda_arm_controller
```

**MoveIt2 configuration files**:
- `panda.srdf` — motion groups, end-effector, collision rules
- `kinematics.yaml` — KDL solver settings
- `ompl_planning.yaml` — planner algorithm settings
- `moveit_controllers.yaml` — controller interface
- `joint_limits.yaml` — max speed/acceleration per joint

---

### Node 6: `planning_scene_loader.py` — Collision Object Manager (Legacy)

**What it is**: Loads collision objects (table, cylinder) from a YAML file and publishes them to MoveIt2's planning scene.

> **Note**: In the current codebase, `pick_place_node.py` handles collision objects inline. This node exists as a standalone alternative.

```
scene_objects.yaml
        │
        ▼
  Parse each object definition
  Build CollisionObject message
        │
        ▼
  Publish to /planning_scene
        │
        ▼
  move_group receives it →
  "Now I know not to plan paths through the table"
```

---

## 6. Every Topic, Message Type & Sample Data

### What is a "topic"?

In ROS2, nodes talk to each other through **topics** — like radio channels. A node **publishes** (broadcasts) to a topic, and other nodes **subscribe** (tune in).

```
Publisher → [/topic_name] → Subscriber(s)
```

---

### `/rgbd_camera/image`

| Field | Value |
|---|---|
| Message Type | `sensor_msgs/msg/Image` |
| Publisher | Gazebo (via ros_gz_bridge) |
| Subscribers | `pose_estimator`, `yolo_detector`, `vlm_detector` |
| Rate | ~15 Hz |

**Sample message structure**:
```
header:
  stamp:
    sec: 12
    nanosec: 450000000
  frame_id: "rgbd_camera_link"
height: 480
width: 640
encoding: "rgb8"       ← 3 bytes per pixel (Red, Green, Blue)
is_bigendian: 0
step: 1920             ← 640 pixels × 3 bytes
data: [214, 198, 176, 215, 199, 177, ...]   ← raw pixel bytes (640×480×3 = 921,600 bytes)
```

**What it looks like** (conceptually):
```
Top-down view of table with boxes:
┌──────────────────────────────┐
│   (gray background)          │
│                              │
│   ┌──────┐  ┌──────┐        │
│   │ RED  │  │ BLUE │        │
│   │ BOX  │  │ BOX  │        │
│   └──────┘  └──────┘        │
│                              │
│         TABLE                │
└──────────────────────────────┘
```

---

### `/rgbd_camera/depth_image`

| Field | Value |
|---|---|
| Message Type | `sensor_msgs/msg/Image` |
| Publisher | Gazebo (via ros_gz_bridge) |
| Subscribers | `yolo_detector`, `vlm_detector` |
| Rate | ~15 Hz |

**Sample message structure**:
```
header: ...
height: 480
width: 640
encoding: "32FC1"      ← 32-bit float, 1 channel (depth in meters)
step: 2560             ← 640 × 4 bytes
data: [...]            ← each 4-byte group = float32 distance in meters
```

**What the values mean** (simplified 5×5 grid):
```
Pixel (u,v) →  depth value (meters from camera)

          u=200  u=280  u=320  u=360  u=440
v=200  [  1.00   1.00   1.00   1.00   1.00 ]  ← background (1m below camera)
v=240  [  1.00   0.58   0.58   0.58   1.00 ]  ← top of red box
v=280  [  1.00   0.58   0.58   0.58   1.00 ]  ← top of red box
v=320  [  1.00   1.00   0.60   1.00   1.00 ]  ← table surface
v=360  [  1.00   1.00   1.00   1.00   1.00 ]  ← table surface
```

---

### `/rgbd_camera/camera_info`

| Field | Value |
|---|---|
| Message Type | `sensor_msgs/msg/CameraInfo` |
| Publisher | Gazebo (via ros_gz_bridge) |
| Subscribers | `pose_estimator`, `yolo_detector`, `vlm_detector` |
| Rate | ~15 Hz |

**Sample message structure**:
```
header:
  frame_id: "rgbd_camera_link"
height: 480
width: 640
distortion_model: "plumb_bob"
d: [0.0, 0.0, 0.0, 0.0, 0.0]   ← no lens distortion (simulated camera)
k: [554.3, 0.0, 320.0,          ← Camera intrinsic matrix K
     0.0, 554.3, 240.0,         ← fx=554.3, fy=554.3 (focal lengths in pixels)
     0.0,   0.0,   1.0]         ← cx=320, cy=240 (principal point = image center)
```

The **K matrix** tells you how the 3D world maps to 2D pixels:
```
K = [fx   0   cx]   [554.3   0   320.0]
    [ 0  fy   cy] = [  0   554.3 240.0]
    [ 0   0    1]   [  0     0     1  ]

fx, fy = focal lengths (pixels)
cx, cy = optical center (usually image center = 320, 240)
```

---

### `/detected_pose`

| Field | Value |
|---|---|
| Message Type | `geometry_msgs/msg/PoseStamped` |
| Publisher | `pose_estimator` OR `yolo_detector` OR `vlm_detector` |
| Subscriber | `pick_place_node` |
| Rate | Published once on first detection |

**Sample message structure**:
```
header:
  stamp:
    sec: 15
    nanosec: 230000000
  frame_id: "world"        ← pose is in world coordinates
pose:
  position:
    x: 0.600               ← 60cm in front of robot base
    y: -0.120              ← 12cm to the right
    z: 0.440               ← 44cm high (on top of table)
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0                 ← identity quaternion (no rotation needed for position-only)
```

**This is the key message** that connects vision to motion — it tells `pick_place_node` exactly where the box is.

---

### `/planning_scene`

| Field | Value |
|---|---|
| Message Type | `moveit_msgs/msg/PlanningScene` |
| Publisher | `pick_place_node` |
| Subscriber | `move_group` |
| When | 2–3 times during pick-place sequence |

**Sample message structure** (adding the table):
```
name: ""
is_diff: true             ← "this is a change, not a full replacement"
world:
  collision_objects:
    - header:
        frame_id: "world"
      id: "table"
      primitives:
        - type: 1          ← BOX (1=box, 2=sphere, 3=cylinder, 4=cone)
          dimensions: [0.4, 0.8, 0.4]   ← [x_size, y_size, z_size] in meters
      primitive_poses:
        - position:
            x: 0.7
            y: 0.0
            z: 0.2         ← center of box at z=0.2 (half the 0.4m height)
          orientation:
            w: 1.0
      operation: 0         ← ADD (0=add, 1=remove, 2=append)
```

**Why this matters**: MoveIt2 uses this to avoid planning paths that go through the table or other obstacles.

---

### `/hand_trajectory_controller/joint_trajectory`

| Field | Value |
|---|---|
| Message Type | `trajectory_msgs/msg/JointTrajectory` |
| Publisher | `pick_place_node` |
| Subscriber | `hand_trajectory_controller` |
| When | Open gripper (before grasp), Close gripper (after descend) |

**Sample message — Opening the gripper**:
```
header:
  stamp: ...
joint_names: ['panda_finger_joint1']   ← only control joint1, joint2 mirrors it
points:
  - positions: [0.04]      ← 4cm = fully open
    velocities: [0.0]
    time_from_start:
      sec: 2
      nanosec: 0
```

**Sample message — Closing the gripper**:
```
joint_names: ['panda_finger_joint1']
points:
  - positions: [0.0]       ← 0cm = fully closed
    time_from_start:
      sec: 2
      nanosec: 0
```

**Why only joint1?** The Panda hand has two fingers. `panda_finger_joint2` is a **mimic joint** — Gazebo automatically makes it copy `joint1`. So you only need to command one.

---

### `/joint_states`

| Field | Value |
|---|---|
| Message Type | `sensor_msgs/msg/JointState` |
| Publisher | `joint_state_broadcaster` (controller) |
| Subscribers | `move_group`, RViz, TF system |
| Rate | 100 Hz |

**Sample message** (arm in home position):
```
header:
  stamp: ...
name: ['panda_joint1', 'panda_joint2', 'panda_joint3',
       'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
       'panda_finger_joint1', 'panda_finger_joint2']

position: [0.0,    -0.785,  0.0,    -2.356,  0.0,    1.571,   0.785,
           0.04,    0.04]
           ↑ joint1  ↑ joint2         ↑ joint4         ↑ joint7  ↑ fingers

velocity: [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort:   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

This is like the robot's "proprioception" — it always knows its own joint angles.

---

### `/panda_arm_controller/follow_joint_trajectory` (Action)

| Field | Value |
|---|---|
| Message Type | `control_msgs/action/FollowJointTrajectory` |
| Publisher | `move_group` (MoveIt2) |
| Subscriber | `panda_arm_controller` |
| When | Every arm movement command |

**Sample goal** (simplified):
```
trajectory:
  joint_names: ['panda_joint1', ..., 'panda_joint7']
  points:
    - positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]   ← home
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [-0.12, -0.91, 0.15, -2.1, 0.05, 1.8, 0.85]     ← waypoint 1
      time_from_start: {sec: 1, nanosec: 500000000}
    - positions: [-0.15, -0.95, 0.18, -2.15, 0.06, 1.82, 0.87]   ← waypoint 2
      time_from_start: {sec: 2, nanosec: 200000000}
    ... (typically 50-200 waypoints for a smooth trajectory)
    - positions: [target joint angles]                              ← goal
      time_from_start: {sec: 5, nanosec: 0}
```

---

### `/clock`

| Field | Value |
|---|---|
| Message Type | `rosgraph_msgs/msg/Clock` |
| Publisher | Gazebo (via ros_gz_bridge) |
| Subscribers | All nodes (simulation time) |

```
clock:
  sec: 42
  nanosec: 356000000
```

All nodes use simulation time (`use_sim_time: true`) so timers and sleeps are synchronized with Gazebo.

---

### Complete Topic Reference Table

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│ TOPIC                                    │ TYPE                │ PUB → SUB       │
├──────────────────────────────────────────┼─────────────────────┼─────────────────┤
│ /rgbd_camera/image                       │ Image               │ Gazebo → Vision │
│ /rgbd_camera/depth_image                 │ Image               │ Gazebo → Vision │
│ /rgbd_camera/camera_info                 │ CameraInfo          │ Gazebo → Vision │
│ /detected_pose                           │ PoseStamped         │ Vision → Pick   │
│ /joint_states                            │ JointState          │ Controllers → * │
│ /planning_scene                          │ PlanningScene       │ Pick → MoveGroup│
│ /hand_trajectory_controller/             │ JointTrajectory     │ Pick → Gripper  │
│   joint_trajectory                       │                     │   Controller    │
│ /panda_arm_controller/                   │ FollowJoint         │ MoveGroup →     │
│   follow_joint_trajectory (action)       │ Trajectory          │   Arm Controller│
│ /clock                                   │ Clock               │ Gazebo → All    │
│ /tf                                      │ TFMessage           │ RSP → All       │
│ /tf_static                               │ TFMessage           │ RSP → All       │
└──────────────────────────────────────────┴─────────────────────┴─────────────────┘

RSP = Robot State Publisher
* = MoveGroup, RViz, TF system
```

---

## 7. End-to-End Scenario: "Pick the Blue Box"

**Setup**: Run with `detector:=vlm` (OWL-ViT phase 2C), target = "blue box"

```bash
ros2 launch pick_place_control pick_place_demo.launch.py detector:=vlm
```

---

### T=0s — System Starts

```
[Gazebo launches]
  → Loads pick_place.sdf world
  → Spawns Panda robot
  → Starts RGBD camera
  → Starts physics simulation

[arm_bringup controllers spawn]
  → joint_state_broadcaster starts
  → panda_arm_controller starts
  → hand_trajectory_controller starts

[move_group starts]
  → Loads URDF + SRDF
  → Initializes KDL kinematics solver
  → Initializes OMPL planner
  → Starts listening on /planning_scene

[vlm_detector starts]
  → Downloads OWL-ViT model (or loads from cache)
  → target_object = "blue box"
  → Subscribes to /rgbd_camera/image
  → Subscribes to /rgbd_camera/depth_image
  → Subscribes to /rgbd_camera/camera_info
```

---

### T=1–8s — Vision Detects the Box

```
[Gazebo RGBD Camera publishes at 15 Hz]

/rgbd_camera/image       → [RGB image: top-down view of table with blue box]
/rgbd_camera/depth_image → [Depth map: values in meters]
/rgbd_camera/camera_info → [K matrix: fx=554.3, fy=554.3, cx=320, cy=240]

                    │
                    ▼
         [vlm_detector receives images]

Step 1: Convert ROS image to numpy array
  arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(480, 640, 3)

Step 2: OWL-ViT inference
  inputs = processor(text=[["blue box"]], images=pil_image)
  outputs = owl_model(**inputs)
  → scores: [0.82]                    ← 82% confident this is "blue box"
  → boxes:  [[247, 188, 393, 292]]    ← bounding box [x1,y1,x2,y2] in pixels

Step 3: Compute center
  u = (247 + 393) / 2 = 320
  v = (188 + 292) / 2 = 240

Step 4: Sample depth
  depth = depth_image[240, 320] = 0.560  ← box top is 56cm below camera

Step 5: Back-project to 3D camera-optical frame
  x_cam = (320 - 320) × 0.560 / 554.3 = 0.000
  y_cam = (240 - 240) × 0.560 / 554.3 = 0.000
  z_cam = 0.560

Step 6: Transform camera-optical → world
  Camera at world (0.6, 0.0, 1.0), pitch=90°
  p_world = [0.600, -0.120, 0.440]   ← blue box at x=0.6, y=-0.12, z=0.44

Step 7: Publish
  /detected_pose → PoseStamped
    position: {x: 0.600, y: -0.120, z: 0.440}
    frame_id: "world"
```

---

### T=8s — pick_place_node Activates (8s delay for move_group startup)

```
[pick_place_node receives /detected_pose]
  → Stores box_position = [0.600, -0.120, 0.440]
  → vlm_detector stops publishing (first detection only)
  → Begins pick-place sequence
```

---

### T=8s — Step 1: Move to Home

```
pick_place_node calls:
  moveit2.move_to_configuration(HOME_JOINT_POSITIONS)
  moveit2.wait_until_executed()

HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

[move_group receives request]
  → KDL IK: not needed (joint-space goal, not Cartesian)
  → OMPL plans path from current joints to HOME joints
  → Sends trajectory to panda_arm_controller

[panda_arm_controller receives trajectory]
  → Sends position commands to Gazebo at 100Hz
  → Arm smoothly moves to home pose
  → Publishes /joint_states throughout

[After ~3s]
  → move_group reports: SUCCEEDED
  → pick_place_node continues
```

```
  Robot in home position:
         ___
        /   \
       |  ●  |   ← camera overhead
        \___/
           |
      ┌────┴────┐
      │  TABLE  │  [RED] [BLUE]
      └─────────┘
   BASE
     \
      ⌒⌒⌒   ← arm folded in "ready" pose
```

---

### T=11s — Step 2: Load Planning Scene (Collision Objects)

```
pick_place_node publishes to /planning_scene:

Object 1: TABLE (box)
  id="table"
  dimensions: [0.4, 0.8, 0.4]   ← 40cm×80cm×40cm
  position: [0.7, 0.0, 0.2]

Object 2: BLUE_CYLINDER (obstacle)
  id="blue_cylinder"
  type=CYLINDER
  dimensions: [0.08, 0.02]      ← height=8cm, radius=2cm
  position: [0.7, 0.2, 0.44]

[move_group receives PlanningScene]
  → "Now I know: table is a box from z=0 to z=0.4.
     I will NOT plan paths through z=0 to z=0.4 in that region."
```

```
  Planning scene (side view):
  z=1.0  [camera]
  z=0.7
  z=0.52 [PRE-GRASP target]
  z=0.44 [box top / grasp target]
  z=0.40 ════════════════════ TABLE TOP
  z=0.20 [table center]
  z=0.00 ════════════════════ GROUND
         x=0.6     x=0.7
```

---

### T=11.5s — Step 3: Open Gripper

```
pick_place_node publishes to:
  /hand_trajectory_controller/joint_trajectory

Message:
  joint_names: ['panda_finger_joint1']
  points:
    - positions: [0.04]    ← 4cm apart = open
      time_from_start: 2s

[hand_trajectory_controller]
  → Commands panda_finger_joint1 to 0.04
  → Gazebo mirrors to panda_finger_joint2 automatically
  → Gripper opens wide

[After 2.5s]
  → pick_place_node sleeps 2.5s to let gripper finish
```

---

### T=14s — Step 4: Move to Pre-Grasp Position

```
Compute pre-grasp position:
  box is at [0.600, -0.120, 0.440]
  PRE_GRASP_HEIGHT = 0.15m above box

  pre_grasp_z = detected_z - BOX_HALF_HEIGHT + HAND_LENGTH + PRE_GRASP_HEIGHT
              = 0.440 - 0.02 + 0.12 + 0.15
              = 0.69  m

  pre_grasp = [0.600, -0.120, 0.690]
  orientation = [0, 1, 0, 0]   ← gripper pointing straight down

pick_place_node calls:
  moveit2.move_to_pose(
      position=[0.600, -0.120, 0.690],
      quat_xyzw=[0, 1, 0, 0],
      cartesian=True           ← straight line path
  )

[move_group receives request]
  → KDL IK: finds joint angles for (0.6, -0.12, 0.69) pointing down
  → OMPL plans collision-free path checking TABLE and CYLINDER
  → Sends trajectory to panda_arm_controller

[panda_arm_controller executes]
  → Arm moves to hover position above blue box
```

---

### T=19s — Step 5: Descend to Grasp

```
Compute grasp position:
  grasp_z = detected_z - BOX_HALF_HEIGHT + HAND_LENGTH
          = 0.440 - 0.02 + 0.12
          = 0.54  m

  grasp = [0.600, -0.120, 0.540]

pick_place_node calls:
  moveit2.move_to_pose(
      position=[0.600, -0.120, 0.540],
      quat_xyzw=[0, 1, 0, 0],
      cartesian=True           ← straight down
  )

[move_group plans straight-down Cartesian path]
  → Short descent, fingers will be at z=0.54-0.12=0.42 = box center ✓

[After ~2s]
  → Arm is positioned with open fingers around the box
```

```
  Finger position breakdown:
  z=0.54 ← panda_link8 (end effector)
  z=0.46 ← hand body
  z=0.42 ← fingertips  ← at box CENTER height ✓
  z=0.40 ← TABLE TOP
  z=0.40 to 0.44 = BOX (0.04m tall box)
```

---

### T=21s — Step 6: Close Gripper (Grasp!)

```
pick_place_node publishes to:
  /hand_trajectory_controller/joint_trajectory

Message:
  joint_names: ['panda_finger_joint1']
  points:
    - positions: [0.0]    ← 0cm = fully closed
      time_from_start: 2s

[hand_trajectory_controller]
  → panda_finger_joint1 → 0.0
  → panda_finger_joint2 mirrors automatically
  → Fingers close around the blue box
```

---

### T=23.5s — Step 7: Attach Box to Arm in Planning Scene

```
pick_place_node sends AttachObject to move_group:
  object_id = "blue_box"
  link_name = "panda_hand"     ← attach to this link

[move_group]
  → Blue box is now part of the robot
  → When planning future paths, the box MOVES WITH the arm
  → Collision checking will avoid the box hitting things
```

---

### T=24s — Step 8: Lift

```
lift_position = [0.600, -0.120, 0.690]   ← same as pre-grasp but now holding box

pick_place_node calls:
  moveit2.move_to_pose(position=lift_position, cartesian=True)

[move_group plans upward path]
  → Box is attached, so planner treats arm+box as one body
  → Moves straight up
```

---

### T=26s — Step 9: Move to Place Position

```
PLACE_POSITION = [0.5, 0.3, 0.54]   ← hard-coded drop-off location

pick_place_node calls:
  moveit2.move_to_pose(position=PLACE_POSITION, quat_xyzw=[0,1,0,0])

[move_group plans arc path]
  → OMPL finds collision-free path to place location
  → Arm swings over carrying the blue box
```

```
Top-down trajectory view:
         PLACE
          ●
         /
        /  (arc path)
       /
      ●  ← pre-grasp/lift position
      |
   TABLE
```

---

### T=30s — Step 10: Detach Box & Release

```
pick_place_node detaches object:
  → move_group removes box from arm's collision body

pick_place_node publishes gripper open:
  → positions: [0.04]   ← open

[Box released]
  → Gazebo physics takes over
  → Box rests at place location

[pick_place_node sleeps 1s for physics to settle]
```

---

### T=31s — Step 11: Retreat & Return Home

```
Retreat position = [0.5, 0.3, 0.69]   ← same x,y but higher z

moveit2.move_to_pose(position=retreat_position, cartesian=True)
moveit2.move_to_configuration(HOME_JOINT_POSITIONS)

[Arm returns to home position]
  → Task complete!
```

---

### Complete Timeline Summary

```
T=0s    ─── System launch (Gazebo, controllers, MoveGroup, VLM detector)
T=1-8s  ─── OWL-ViT detects blue box → publishes /detected_pose
T=8s    ─── pick_place_node activates
T=8-11s ─── HOME move
T=11s   ─── Load planning scene (table + cylinder collision objects)
T=11.5s ─── Open gripper
T=14s   ─── Move to PRE-GRASP (hover above box)
T=19s   ─── DESCEND to grasp position
T=21s   ─── Close gripper
T=23.5s ─── Attach box to arm
T=24s   ─── LIFT
T=26s   ─── Move to PLACE position
T=30s   ─── Detach box, open gripper, release
T=31s   ─── RETREAT
T=33s   ─── Return HOME
T=35s   ─── Task complete ✓
```

---

## 8. Coordinate Systems

This is one of the trickiest parts. There are **three different coordinate frames** in play:

### World Frame (ROS)

```
Origin = robot base (panda_link0 = "world")
X → forward (toward table)
Y → left
Z → up

         Z
         │
         │
         ├─────── Y
        /
       /
      X

Robot base at origin.
Table center at (0.7, 0.0, 0.2).
Blue box at (0.6, -0.12, 0.44).
Camera at (0.6, 0.0, 1.0).
```

### Camera Optical Frame (OpenCV convention)

```
Origin = camera lens
X → right (in image)
Y → down (in image)
Z → forward (into the scene)

       Z (into scene)
      /
     /
    ●────── X (right in image)
    │
    │
    Y (down in image)

This is what solvePnP and back-projection use.
```

### Gazebo Camera Link Frame

```
Origin = camera body
X → forward
Y → left
Z → up

(Same as world frame convention, just at camera's position/rotation)
```

### The Transform Matrix

The camera in the world is at position `(0.6, 0.0, 1.0)` with `pitch = π/2` (tilted 90° to point down).

```
World → Camera-Optical transformation matrix (4×4 homogeneous):

T_world_to_optical = [0   -1   0   0.6]
                     [0    0  -1   0.0]
                     [-1   0   0   1.0]
                     [0    0   0   1.0]

p_world = T × p_camera_optical

Practical example:
  Camera center pixel (u=320, v=240) with depth=0.56m:
  → x_cam=0, y_cam=0, z_cam=0.56
  → Transforms to world (0.6, 0.0, 0.44)
                          ↑ camera x  ↑ table height (1.0 - 0.56)
```

---

## 9. How Controllers Work

```
┌─────────────────────────────────────────────────────────────┐
│                    ros2_control Framework                    │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                 controller_manager                    │  │
│  │  (spawns and manages all controllers)                 │  │
│  └──────────┬──────────────┬──────────────┬─────────────┘  │
│             │              │              │                  │
│    ┌────────▼────────┐  ┌──▼──────────┐  ┌▼──────────────┐ │
│    │joint_state_     │  │panda_arm_   │  │hand_trajectory│ │
│    │broadcaster      │  │controller   │  │_controller    │ │
│    │                 │  │             │  │               │ │
│    │Reads joint      │  │JointTraj-   │  │JointTraj-     │ │
│    │positions from   │  │ectory       │  │ectory         │ │
│    │Gazebo           │  │Controller   │  │Controller     │ │
│    │                 │  │for joints   │  │for fingers    │ │
│    │Publishes:       │  │1-7          │  │               │ │
│    │/joint_states    │  │             │  │Input:         │ │
│    └─────────────────┘  │Input:       │  │/hand_traj_    │ │
│                         │follow_joint │  │controller/    │ │
│                         │_trajectory  │  │joint_traj     │ │
│                         │(action)     │  │(topic)        │ │
│                         └─────────────┘  └───────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

**Key difference**: The arm controller uses an **action** (two-way communication, move_group knows when the motion is done), while the gripper controller uses a simple **topic** (fire and forget, pick_place_node uses a sleep to wait).

**Controller configuration** (`ros2_controllers.yaml`):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

panda_arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints: [panda_joint1, panda_joint2, panda_joint3,
           panda_joint4, panda_joint5, panda_joint6, panda_joint7]
  command_interfaces: [position]
  state_interfaces: [position, velocity]

hand_trajectory_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints: [panda_finger_joint1]
  command_interfaces: [position]
  state_interfaces: [position]
```

---

## 10. Vision Backends Compared

```
┌─────────────────┬────────────────────┬────────────────────┬──────────────────┐
│ Feature         │ Phase 2A AprilTag  │ Phase 2B YOLO      │ Phase 2C OWL-ViT │
├─────────────────┼────────────────────┼────────────────────┼──────────────────┤
│ Uses depth?     │ No (solvePnP)      │ Yes                │ Yes              │
│ Needs training? │ No                 │ Yes (pretrained)   │ No               │
│ Query type      │ Visual marker ID   │ Class name         │ Natural language │
│ Accuracy        │ Very high (mm)     │ Moderate           │ Moderate         │
│ Speed           │ Fast (<5ms)        │ Fast (~20ms)       │ Slow (~200ms)    │
│ Real-world use  │ Industrial/fixtured│ General objects    │ Open-vocabulary  │
│ Fallback        │ None               │ HSV color mask     │ HSV color mask   │
│ Model size      │ OpenCV built-in    │ ~6 MB (yolov8n)    │ ~580 MB          │
│ Object ID       │ Printed marker     │ Predefined classes │ Any text prompt  │
└─────────────────┴────────────────────┴────────────────────┴──────────────────┘
```

**Which to use when**:
- **Phase 2A**: Best accuracy. Use in controlled environments where you can print/place markers.
- **Phase 2B**: Good for known objects (bottles, cups, boxes) with real cameras.
- **Phase 2C**: Best flexibility — say "find the blue mug" without any pre-training.

---

## 11. Final Complete System Diagram

```
╔══════════════════════════════════════════════════════════════════════╗
║                    COMPLETE SYSTEM ARCHITECTURE                      ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║  [GAZEBO SIMULATION]                                                 ║
║  ┌────────────────────────────────────────────────────────────────┐ ║
║  │  pick_place.sdf world:                                         │ ║
║  │   • ground plane                                               │ ║
║  │   • table (x=0.7, z=0.4m)                                     │ ║
║  │   • red_box (AprilTag, Phase2A)                                │ ║
║  │   • red_box_plain (Phase2B YOLO)                               │ ║
║  │   • blue_box (Phase2C VLM)                                     │ ║
║  │   • blue_cylinder (obstacle)                                   │ ║
║  │   • RGBD camera (x=0.6, z=1.0, pitch=90°)                    │ ║
║  │   • Panda robot (base at origin)                               │ ║
║  └──────────┬──────────────────────────────────┬──────────────────┘ ║
║             │ gz_ros_bridge                    │ gz_ros2_control     ║
║             │                                  │                     ║
║  ┌──────────▼───────────────────────────┐      │                     ║
║  │  Camera Topics (Gazebo → ROS2)       │      │                     ║
║  │  • /rgbd_camera/image   (Image 15Hz) │      │                     ║
║  │  • /rgbd_camera/depth_image (15Hz)   │      │                     ║
║  │  • /rgbd_camera/camera_info (15Hz)   │      │                     ║
║  │  • /clock (sim time)                 │      │                     ║
║  └──────────┬───────────────────────────┘      │                     ║
║             │                                  │                     ║
║  ┌──────────▼───────────────────────────┐  ┌───▼──────────────────┐ ║
║  │  VISION NODE (one of three)          │  │   CONTROLLERS        │ ║
║  │                                      │  │   (ros2_control)     │ ║
║  │  pose_estimator.py   (AprilTag 2A)   │  │                      │ ║
║  │  yolo_detector.py    (YOLO 2B)       │  │ ┌──────────────────┐ │ ║
║  │  vlm_detector.py     (OWL-ViT 2C)   │  │ │joint_state_bcast │ │ ║
║  │                                      │  │ │→ /joint_states   │ │ ║
║  │  output:                             │  │ └──────────────────┘ │ ║
║  │  /detected_pose (PoseStamped)        │  │                      │ ║
║  └──────────┬───────────────────────────┘  │ ┌──────────────────┐ │ ║
║             │                              │ │panda_arm_ctrl    │ │ ║
║  ┌──────────▼───────────────────────────┐  │ │← follow_joint    │ │ ║
║  │  pick_place_node.py (Orchestrator)   │  │ │  _trajectory     │ │ ║
║  │                                      │  │ └──────────────────┘ │ ║
║  │  1. Waits for /detected_pose         │  │                      │ ║
║  │  2. Sequences pick-place steps       │  │ ┌──────────────────┐ │ ║
║  │  3. Calls pymoveit2 API for arm      │  │ │hand_traj_ctrl    │ │ ║
║  │  4. Publishes gripper commands       │  │ │← /hand_traj_ctrl │ │ ║
║  │  5. Manages planning scene           │◄─┼─┤  /joint_traj     │ │ ║
║  └──────────┬──────────────┬────────────┘  │ └──────────────────┘ │ ║
║             │              │               └──────────┬────────────┘ ║
║             │ pymoveit2    │ /planning_scene           │              ║
║             │ (MoveIt2     │ (collision objects)       │ /joint_states║
║             │  API calls)  │                           │              ║
║  ┌──────────▼──────────────▼────────────┐              │              ║
║  │  move_group (MoveIt2)                │◄─────────────┘              ║
║  │                                      │                             ║
║  │  • KDL IK solver                     │                             ║
║  │  • OMPL motion planner               │                             ║
║  │  • Collision checker                 │                             ║
║  │  • Planning scene manager            │                             ║
║  │                                      │                             ║
║  │  → follow_joint_trajectory (action)  │                             ║
║  └──────────────────────────────────────┘                             ║
╚══════════════════════════════════════════════════════════════════════╝
```

---

## Summary

Each component has a clear, single responsibility:

| Component | Role | Analogy |
|---|---|---|
| **Gazebo** | Simulated physics world | A video game world |
| **RGBD Camera** | Eyes of the system | Security camera |
| **Vision nodes** | Interpret what the camera sees | Image recognition app |
| **pick_place_node** | Decides what to do and when | A chef following a recipe |
| **move_group** | Plans safe motion paths | GPS route planner |
| **Controllers** | Actually move the joints | Servo motor drivers |
| **Topics** | How all components talk | Radio channels |
| **Planning scene** | Map of obstacles | Map for the GPS |
| **SRDF/URDF** | Defines the robot's structure | Robot's anatomy + rules |

---

## 11. Deep Dive: YOLO → pick_place_node Function-by-Function

This section traces the complete journey from a camera frame arriving in `yolo_detector.py` all the way to the robot arm executing every movement in `pick_place_node.py`, function by function in order.

---

### Phase 1 — YOLO Starts Up

**`YoloDetector.__init__`** (`yolo_detector.py:34`)

Three subscriptions are created and one publisher:

```
Subscribes to:
  /rgbd_camera/camera_info  → _camera_info_cb
  /rgbd_camera/image        → _image_cb          ← main loop, called every frame
  /rgbd_camera/depth_image  → _depth_cb

Publishes to:
  /detected_pose  (PoseStamped)
```

Also calls **`_build_camera_transform`** (`yolo_detector.py:76`) once at startup — precomputes the 4×4 matrix that converts camera-optical coordinates → world coordinates. Done once here so it doesn't recalculate every frame.

---

### Phase 2 — Camera Data Arrives (~15 Hz)

**`_camera_info_cb`** (`yolo_detector.py:101`)

Called once (returns early after first message). Stores the `K` matrix:
```
fx=554.3, fy=554.3, cx=320, cy=240
```
These are needed later to back-project pixels into 3D space.

**`_depth_cb`** (`yolo_detector.py:108`)

Called every frame. Just caches the latest depth image as a float32 numpy array (values = meters from camera). Keeps overwriting — only the latest one matters.

**`_image_cb`** (`yolo_detector.py:112`)

The main loop — called every RGB frame. Won't proceed unless both `camera_matrix` and `depth_image` are already available (guard at line 114).

---

### Phase 3 — Object Detection

**`_detect_object`** (`yolo_detector.py:172`) — called from `_image_cb`.

Returns a single `(u, v)` pixel coordinate of the object center, or `None`.

**Path 1 — YOLO succeeds** (`yolo_detector.py:181`):
```python
results = self.yolo(frame, verbose=False, conf=self.conf_threshold)
# Loops detections, finds matching target_class ('bottle')
# Returns center of bounding box:
cx = (x1 + x2) / 2.0
cy = (y1 + y2) / 2.0
```

**Path 2 — YOLO fails → HSV color fallback** (`yolo_detector.py:199`):

YOLO doesn't reliably recognize Gazebo's plain red box as a "bottle", so:
```
RGB image → HSV colorspace
→ mask pixels in red hue range [0-10] and [160-180]
  (red wraps around in HSV — needs two ranges)
→ find contours (blobs of red pixels)
→ pick the largest contour
→ compute centroid using image moments
→ return that centroid as (cx, cy)
```

---

### Phase 4 — Pixel → 3D World Position

Back in **`_image_cb`** (`yolo_detector.py:124`):

**Step 1 — Sample depth** (`yolo_detector.py:128`):
```python
depth = float(self.depth_image[v, u])
# e.g., 0.580  ← the box top is 58cm below the camera
```

**Step 2 — Back-project pixel to camera frame** (`yolo_detector.py:140`):
```python
x_cam = (u - cx) * depth / fx     # how far left/right of camera center
y_cam = (v - cy) * depth / fy     # how far up/down of camera center
z_cam = depth                      # how far forward from the lens
```
This reverses the pinhole camera projection. If the pixel is exactly at `(cx, cy)` — dead center — then `x_cam=0, y_cam=0`, and the point is directly below the camera.

**Step 3 — Transform to world frame** (`yolo_detector.py:146`):
```python
p_cam_h = np.array([x_cam, y_cam, z_cam, 1.0])
p_world = self.T_world_to_optical @ p_cam_h
# T_world_to_optical built in __init__ using camera's known pose from the SDF
```

**Step 4 — Publish** (`yolo_detector.py:152`):
```python
pose_msg = PoseStamped()
pose_msg.header.frame_id = 'world'
pose_msg.pose.position.x = p_world[0]   # e.g., 0.600
pose_msg.pose.position.y = p_world[1]   # e.g., 0.120
pose_msg.pose.position.z = p_world[2]   # e.g., 0.440
pose_msg.pose.orientation.w = 1.0       # identity — pick_place_node ignores orientation
self.pose_pub.publish(pose_msg)
```

> YOLO publishes this **every frame at 15 Hz** — continuously overwriting — until pick_place_node consumes it.

---

### Phase 5 — pick_place_node Wakes Up

**`PickPlaceNode.__init__`** (`pick_place_node.py:41`)

On startup it:
1. Creates the `MoveIt2` interface (`pick_place_node.py:50`) — the pymoveit2 object that talks to `move_group`
2. Creates the gripper publisher (`pick_place_node.py:66`) — direct topic to the hand controller
3. Subscribes to `/detected_pose` → **`_detected_pose_cb`** (`pick_place_node.py:76`)
4. Starts a 1 Hz timer → **`_wait_for_detection`** (`pick_place_node.py:81`)

**`_detected_pose_cb`** (`pick_place_node.py:86`):

Dead simple — just stores whatever YOLO most recently published:
```python
def _detected_pose_cb(self, msg):
    self.detected_pose = msg
```

**`_wait_for_detection`** (`pick_place_node.py:89`):

Called every 1 second by the timer. The moment `self.detected_pose` is not `None`:
```python
self._wait_timer.cancel()   # stop the 1Hz polling
self.run_pick_and_place()   # hand off to the sequence
```

---

### Phase 6 — The Pick-Place Sequence

**`run_pick_and_place`** (`pick_place_node.py:99`)

First extracts position from what YOLO sent and computes the three key Z heights:

```python
tag = self.detected_pose.pose.position
# tag.x, tag.y, tag.z  ← from YOLO (e.g., 0.6, 0.12, 0.44)

grasp_z     = tag.z - BOX_HALF_HEIGHT + HAND_LENGTH
#           = 0.44  -     0.02        +    0.12       = 0.54 m
#
# Why? YOLO sees the BOX TOP (z=0.44).
# Subtract half box height to get box CENTER (z=0.42).
# Add HAND_LENGTH (0.12) because panda_link8 sits 12cm ABOVE the fingertips.
# So panda_link8 must be at z=0.54 for fingertips to reach z=0.42 (box center).

pre_grasp_z = grasp_z + PRE_GRASP_HEIGHT
#           =  0.54  +      0.15          = 0.69 m  ← safe hover height
```

Then calls helpers in order:

| Step | Function | File:Line | cartesian | What happens |
|---|---|---|---|---|
| 1 | `_move_to_home` | `pick_place_node.py:161` | — | Joint-space move to home angles |
| 2 | `_load_planning_scene` | `pick_place_node.py:215` | — | Publish table + cylinder to `/planning_scene` |
| 3 | `_open_gripper` → `_move_gripper(0.04)` | `pick_place_node.py:191` | — | Publish gripper topic, sleep 2.5s |
| 4 | `_move_to_pose("PRE-GRASP")` | `pick_place_node.py:167` | False | OMPL plans arc to hover point |
| 5 | `_move_to_pose("GRASP")` | `pick_place_node.py:167` | **True** | Straight-line descent |
| 6 | `_close_gripper` → `_move_gripper(0.0)` | `pick_place_node.py:194` | — | Close fingers, sleep 2.5s |
| 7 | `_attach_object` | `pick_place_node.py:197` | — | Tell MoveIt2: box is part of the arm |
| 8 | `_move_to_pose("LIFT")` | `pick_place_node.py:167` | **True** | Straight-line up |
| 9 | `_move_to_pose("PLACE")` | `pick_place_node.py:167` | False | OMPL plans arc to `[0.5, 0.3, 0.54]` |
| 10 | `_detach_object` | `pick_place_node.py:210` | — | Tell MoveIt2: box is no longer attached |
| 11 | `_open_gripper` | `pick_place_node.py:191` | — | Release the box |
| 12 | `time.sleep(1.0)` | `pick_place_node.py:149` | — | Wait for Gazebo physics |
| 13 | `_move_to_pose("RETREAT")` | `pick_place_node.py:167` | **True** | Straight-line up from place |
| 14 | `_move_to_home` | `pick_place_node.py:161` | — | Return to home joints |

---

### The Two Move Types — Critical Distinction

**`_move_to_home`** (`pick_place_node.py:161`) calls:
```python
self.moveit2.move_to_configuration(HOME_JOINT_POSITIONS)
```
**Joint-space goal.** You specify exact joint angles. MoveIt2 plans any path to reach them — usually an arc. Used only for home because the exact Cartesian path doesn't matter there.

**`_move_to_pose`** (`pick_place_node.py:167`) calls:
```python
self.moveit2.move_to_pose(position=..., quat_xyzw=..., cartesian=cartesian)
```
- **`cartesian=False`** → OMPL plans freely — arc, curve, anything collision-free. Used for PRE-GRASP and PLACE where you have room to maneuver and don't care about the path shape.
- **`cartesian=True`** → MoveIt2 forces a **straight line in XYZ**. Used for GRASP, LIFT, and RETREAT because vertical-only motion avoids knocking the box sideways or hitting the table.

Every move ends with:
```python
self.moveit2.wait_until_executed()
```
This **blocks** until `move_group` confirms the trajectory completed — only then does the next step begin. This is what makes the sequence safe and sequential.

---

### The Gripper — Why It's Different

**`_move_gripper`** (`pick_place_node.py:177`) doesn't use MoveIt2 at all:

```python
traj = JointTrajectory()
traj.joint_names = ['panda_finger_joint1']
pt = JointTrajectoryPoint()
pt.positions = [position]           # 0.04 = open, 0.0 = closed
pt.time_from_start = Duration(sec=2)
traj.points = [pt]
self._gripper_pub.publish(traj)     # fire-and-forget topic publish
time.sleep(2.5)                     # only way to "wait" — no feedback
```

This bypasses MoveIt2 entirely. It publishes directly to `/hand_trajectory_controller/joint_trajectory` and then sleeps. There is no success/failure feedback — the sleep is the only synchronization.

**Why?** pymoveit2's `MoveIt2Gripper` class has a bug — it looks for a `panda_hand_tcp` link that doesn't exist in this URDF. The direct topic approach bypasses all of that complexity.

---

### Complete YOLO → pick_place_node Data Flow

```
yolo_detector.py                      pick_place_node.py
────────────────                      ──────────────────
__init__
  _build_camera_transform()            __init__
  YOLO model loaded                      MoveIt2 interface created
  subscribe to camera topics             subscribe to /detected_pose → _detected_pose_cb
                                         1Hz timer → _wait_for_detection

_camera_info_cb  → stores K matrix
_depth_cb        → caches depth img (every frame)
_image_cb        → (every frame)
  _detect_object()
    → YOLO inference
    → if fail: HSV color fallback
    → returns (u, v) pixel
  sample depth at (u, v)
  back-project → (x_cam, y_cam, z_cam)
  transform → (x_world, y_world, z_world)
  publish /detected_pose ──────────────► _detected_pose_cb
                                           self.detected_pose = msg (stored)

  [repeats at 15Hz, overwriting]         _wait_for_detection (every 1s)
                                           detected? yes →
                                           timer.cancel()
                                           run_pick_and_place()
                                             compute grasp_z, pre_grasp_z
                                             _move_to_home()
                                             _load_planning_scene()
                                             _open_gripper()
                                             _move_to_pose("PRE-GRASP", cartesian=False)
                                             _move_to_pose("GRASP",     cartesian=True)
                                             _close_gripper()
                                             _attach_object()
                                             _move_to_pose("LIFT",      cartesian=True)
                                             _move_to_pose("PLACE",     cartesian=False)
                                             _detach_object()
                                             _open_gripper()
                                             time.sleep(1.0)
                                             _move_to_pose("RETREAT",   cartesian=True)
                                             _move_to_home()
                                             → DONE
```

The key insight: **YOLO's only job is to produce a world-frame `(x, y, z)`.** Everything else — the height offsets, IK solving, trajectory planning, gripper timing, collision management — is handled by `pick_place_node` and `move_group`. YOLO and `pick_place_node` are completely decoupled except for the single `/detected_pose` topic.
