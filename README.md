# FlexBE States and Behaviors for GraspSAM

FlexBE service states and behavior pipelines for integrating **GraspSAM** into a ROS 2 manipulation workflow.

This repository provides:

- A **FlexBE service state** for calling a ROS 2 GraspSAM server
- A **FlexBE behavior pipeline** that connects **Unseen Object Clustering (UOC)** as the perception/segmentation front-end
- A recommended perception-to-action flow:
  - **UOC segmentation (RGB-D)**
  - **GraspSAM grasp generation**
  - **MoveIt / OMPL motion planning**

## Overview

This package is a **FlexBE-based integration layer** for GraspSAM in ROS 2.

It currently provides:

1. **`graspsam_service_state.py`** (main state)
   - Calls the ROS 2 GraspSAM service (`/run_graspsam`)
   - Sends GraspSAM runtime arguments (dataset path/name, checkpoint, encoder, grasp count, seen/unseen flag)
   - Receives grasp results and extracts **`pose_base`** fields into `grasp_target_poses` for downstream motion planning

2. **`unseenobjclustergraspsampipeine_sm.py`** (main behavior)
   - Runs **UOC RGB-D segmentation**
   - Selects a target instance / scene mapping
   - Calls **GraspSAM**
   - Sends grasp pose candidates to **MoveIt** through a move-to-pose service

## Recommended Pipeline

**Most recommended:**  
**UOC segmentation + GraspSAM + MoveIt OMPL**

Implemented in:

- `unseenobjclustergraspsampipeine_sm.py`

High-level flow:

1. Segment scene with **Unseen Object Clustering** (`/segmentation_rgbd`)
2. Select target instance and map to GraspSAM scene convention
3. Call **GraspSAM** (`/run_graspsam`)
4. Extract candidate base-frame grasp poses (`grasp_target_poses`)
5. Execute with **MoveToPoseServiceState** (`/move_to_pose`)

## Prerequisite

- Clone the **source code** for GraspSAM under [compare_GraspSAM](https://github.com/zhaohuajing/compare_GraspSAM/tree/master).
- Clone the **ROS 2 Server** for GraspSAM under [GraspSam_ros2](https://github.com/zhaohuajing/GraspSam_ros2), which can be built into a `graspsam_ros2` package that includes the msg `RunGraspSAM.srv` and two ROS 2 servers:
  - `graspsam_server.py` for test run without real-time inputs
  - `graspsam_cam2pose_server.py` for integration with **real-time RGB-D inputs** and optionally **segmentation masks**
- Setup the **Docker environment** using the Dockerfile under [graspsam_docker](https://github.com/zhaohuajing/GraspSam_ros2/blob/main/graspsam_docker/Dockerfile).

## Repository Structure

```text
├── gsam_flexbe
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   └── package.xml
├── gsam_flexbe_behaviors
│   ├── bin
│   │   └── copy_behavior
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── config
│   │   └── example.yaml
│   ├── gsam_flexbe_behaviors
│   │   ├── __init__.py
│   │   └── unseenobjclustergraspsampipeine_sm.py
│   ├── manifest
│   │   └── unseenobjclustergraspsampipeine.xml
│   ├── package.xml
│   ├── resource
│   │   └── gsam_flexbe_behaviors
│   ├── setup.cfg
│   └── setup.py
├── gsam_flexbe_states
│   ├── CHANGELOG.rst
│   ├── gsam_flexbe_states
│   │   ├── __init__.py
│   │   └── graspsam_service_state.py
│   ├── package.xml
│   ├── resource
│   │   └── gsam_flexbe_states
│   ├── setup.cfg
│   └── setup.py
```

## Quick Start

This section is tailored to the service names used in the uploaded FlexBE state/behavior.

### 1) Build the workspace

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

### 2) Start required ROS 2 servers

The provided behavior (`UnseenObjClusterGraspSamPipeine`) expects these services:

- `/segmentation_rgbd` (UOC segmentation server)
- `/run_graspsam` (GraspSAM ROS 2 server)
- `/move_to_pose` (MoveIt/OMPL execution server)

Consider adding the following nodes to your launch file: 
```text
uoc_rgbd_bringup = Node(
    package="unseen_obj_clst_ros2",
    executable="segmentation_rgbd_server",
    name="segmentation_rgbd_server",
    output="screen",
)

graspsam_bringup = Node(
    package="graspsam_ros2",
    executable="graspsam_server.py",
    name="graspsam_server",
    output="screen",
)
```

### 3) Start FlexBE and run the behavior

Open FlexBE App / onboard execution and run:

- `UnseenObjClusterGraspSamPipeine` (**recommended**)

### 4) Verify services

```bash
ros2 service list | grep -E "segmentation_rgbd|run_graspsam|move_to_pose"
```

## Provided FlexBE State

### `GraspSAMServiceState`
**File:** `gsam_flexbe_states/graspsam_service_state.py`

Calls the GraspSAM ROS 2 service (`/run_graspsam`) and returns parsed grasps for downstream execution.

The state accepts GraspSAM runtime configuration through userdata and extracts base-frame target poses from `resp.grasps[*].pose_base` into `userdata.grasp_target_poses`.

**Inputs (userdata)**
- `dataset_root` (`string`)
- `dataset_name` (`string`)
- `checkpoint_path` (`string`)
- `sam_encoder_type` (`string`, default commonly `vit_t`)
- `no_grasps` (`int`)
- `seen_set` (`bool`, optional; falls back to `seen_set_default`)

**Outputs (userdata)**
- `output_dir` (`string`)
- `grasps` (`graspsam_ros2/Grasp[]`)
- `grasp_target_poses` (`geometry_msgs/Pose[]`) — extracted from `grasp.pose_base`
- `message` (`string`)

**Outcomes**
- `done`
- `failed`

**Default service**
- `/run_graspsam`

## Provided FlexBE Behavior (Pipeline)

### `UnseenObjClusterGraspSamPipeine`
**File:** `gsam_flexbe_behaviors/gsam_flexbe_behaviors/unseenobjclustergraspsampipeine_sm.py`

Pipeline:
1. `UnseenObjSegRGBDServiceState` (`/segmentation_rgbd`)
2. `SelectInstanceToSceneNameState` (maps segmentation result to scene convention)
3. `GraspSAMServiceState` (`/run_graspsam`)
4. `MoveToPoseServiceState` (`/move_to_pose`)

Default userdata configured in the behavior includes:
- `dataset_name = 'from_rgbd'`
- `dataset_root = './datasets/sample_scene_ucn'`
- `checkpoint_path = 'pretrained_checkpoint/mobile_sam.pt'`
- `sam_encoder_type = 'vit_t'`
- `no_grasps = 10`
- `seen_set = False`

Why recommended:
- Clean integration from RGB-D segmentation to grasp generation
- Automatically passes GraspSAM output poses to MoveIt
- Works well with the UOC front-end pipeline

## Tables for Easier Documentation

### State summary

| State file | Main class | Inputs | Outputs | Service called | Notes |
|---|---|---|---|---|---|
| `graspsam_service_state.py` | `GraspSAMServiceState` | `dataset_root`, `dataset_name`, `checkpoint_path`, `sam_encoder_type`, `no_grasps`, `seen_set` | `output_dir`, `grasps`, `grasp_target_poses`, `message` | `/run_graspsam` | Main GraspSAM FlexBE state; extracts `pose_base` for motion planning. |

### Behavior summary

| Behavior (FlexBE) | Main file | Pipeline type | Services used | Recommended |
|---|---|---|---|---|
| `UnseenObjClusterGraspSamPipeine` | `unseenobjclustergraspsampipeine_sm.py` | UOC (RGB-D) -> GraspSAM -> MoveIt | `/segmentation_rgbd`, `/run_graspsam`, `/move_to_pose` | Yes (primary) |

## Architecture

### Recommended architecture (UOC + GraspSAM)

```text
RGB-D Camera
   |
   v
Unseen Object Clustering (ROS 2 service: /segmentation_rgbd)
   |
   v
Target instance selection / scene mapping
   |
   v
GraspSAMServiceState
   |
   v
GraspSAM ROS 2 Server (/run_graspsam)
   |
   v
Grasp results (graspsam_ros2/Grasp[])
   |
   v
Extract pose_base -> grasp_target_poses
   |
   v
MoveIt / OMPL (MoveToPoseServiceState -> /move_to_pose)
   |
   v
Robot motion execution
```

## Dependencies

This repository assumes the following ROS 2 packages/services are available in your workspace:

- **FlexBE** (core + onboard/app tooling)
- **GraspSAM ROS 2 server package** (`graspsam_ros2`)
  - `RunGraspSAM.srv`
  - `/run_graspsam` service (from `graspsam_server.py` or `graspsam_cam2pose_server.py`)
- **MoveIt / motion execution service**
  - `/move_to_pose`
- **UOC segmentation service**
  - `/segmentation_rgbd`
- Companion FlexBE state packages used by the behavior:
  - `compare_flexbe_states` (e.g., `MoveToPoseServiceState`, `SelectInstanceToSceneNameState`)
  - `uoc_flexbe_states` (e.g., `UnseenObjSegRGBDServiceState`)

## Installation

Clone into your ROS 2 workspace `src/` folder:

```bash
cd ~/your_ws/src
git clone <your_repo_url>
```

Build and source:

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

## Notes and Recommendations

- The provided behavior is optimized for **UOC + GraspSAM** integration.
- The default behavior parameters assume a GraspSAM dataset layout compatible with:
  - `dataset_root = ./datasets/sample_scene_ucn`
  - `dataset_name = from_rgbd`
- If you use a different checkpoint or dataset layout, update the behavior userdata or expose them as behavior parameters.
- The behavior file is generated by FlexBE. Manual edits outside `[MANUAL]` tags may be overwritten when regenerated.

## Acknowledgments

This repository builds on:

- GraspSAM
- FlexBE
- ROS 2
- MoveIt
- Unseen Object Clustering (UOC)
