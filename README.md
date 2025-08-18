# CARE: Collision Avoidance via Repulsive Estimation

**CoRL 2025** | [Project Page](https://airlab-sogang.github.io/CARE/) | [Paper](link-to-paper) | [Video](link-to-video)

## 📋 Overview

CARE (Collision Avoidance via Repulsive Estimation) is a plug-and-play safety module that enhances visual navigation models without requiring fine-tuning or additional sensors. By combining monocular depth estimation with reactive planning based on Artificial Potential Fields (APF), CARE significantly improves collision avoidance for robots navigating in out-of-distribution environments.

### Key Features
- 🚀 **Zero-shot deployment** - Works without fine-tuning on new environments
- 🔌 **Plug-and-play** - Compatible with any RGB-based navigation model (ViNT, NoMaD, etc.)
- 📷 **Vision-only** - Requires only RGB camera input, no additional sensors
- 🤖 **Multi-robot support** - Tested on LoCoBot, TurtleBot4, and RoboMaster platforms
- ⚡ **Real-time performance** - Lightweight depth estimation with minimal computational overhead

### Performance Highlights
- **100% collision reduction** in goal-conditioned navigation tasks
- **10.7× improvement** in collision-free travel distance during exploration
- Maintains navigation efficiency while significantly improving safety

## 🛠️ Installation

### Prerequisites
- Ubuntu 20.04 or 22.04
- Python 3.10
- CUDA 11.8 compatible GPU
- Conda package manager
- ROS2 (for robot deployment)
- Two terminal windows (for concurrent PD controller and navigation execution)

### Quick Setup

1. **Create Conda Environment**
```bash
conda create -n CARE python=3.10
conda activate CARE
```

2. **Clone Repository with Submodules**
```bash
git clone --recursive https://github.com/airlab-sogang/CARE.git
cd CARE
```

3. **Install Dependencies**
```bash
# Install conda packages
conda install -c conda-forge opencv einops wandb prettytable -y
              
# Install core dependencies
pip install -e UniDepth/ --extra-index-url https://download.pytorch.org/whl/cu118
pip install -e train/
pip install -e diffusion_policy/

# Install additional packages
pip install rospkg efficientnet_pytorch warmup_scheduler diffusers vit-pytorch
pip install --upgrade "numpy<2,>=1.26"
```

## 🚀 Quick Start

### Important: Two-Terminal Setup Required
**You need to run two programs simultaneously in separate terminals:**

#### Terminal 1: PD Controller (Required)
```bash
# This must be running for any navigation mode
python pd_controller.py --control apf --robot locobot
```

#### Terminal 2: Navigation Mode (Choose one)

**Standard Navigation (Baseline)**
```bash
python explore.py --waypoint 2 --robot locobot
```

**CARE-Enhanced Navigation (Recommended)**
```bash
python explore_care.py --waypoint 2 --robot locobot
```

## 📖 Usage

### ⚠️ Important: Two-Process Architecture
CARE requires running two processes simultaneously:
1. **PD Controller** (Terminal 1) - Handles robot control with APF
2. **Navigation Module** (Terminal 2) - Provides waypoint generation

### Supported Robot Platforms
- **LoCoBot** - 170° FOV fisheye camera
- **TurtleBot4** - 89.5° FOV OAK-D Pro camera
- **RoboMaster S1** - 120° FOV camera

### Navigation Modes

#### 1. Exploration Mode
Autonomous exploration with collision avoidance:

**Terminal 1 (PD Controller - Required):**
```bash
python pd_controller.py --control apf --robot [ROBOT_TYPE]
```

**Terminal 2 (Choose one):**
```bash
# Standard exploration
python explore.py --waypoint 2 --robot [ROBOT_TYPE]

# CARE-enhanced exploration (recommended)
python explore_care.py --waypoint 2 --robot [ROBOT_TYPE]
```

#### 2. Goal-Conditioned Navigation
Navigate to specific image goals:

**Terminal 1 (PD Controller - Required):**
```bash
python pd_controller.py --control apf --robot [ROBOT_TYPE]
```

**Terminal 2:**
```bash
# With goal image
python explore_care.py --waypoint 2 --robot [ROBOT_TYPE] --goal_image path/to/goal.jpg
```

### Complete Usage Examples

#### Example 1: LoCoBot CARE-Enhanced Exploration
```bash
# Terminal 1 (keep running)
python pd_controller.py --control apf --robot locobot

# Terminal 2
python explore_care.py --waypoint 2 --robot locobot
```

#### Example 2: TurtleBot4 Standard Navigation
```bash
# Terminal 1 (keep running)
python pd_controller.py --control apf --robot turtlebot4

# Terminal 2
python explore.py --waypoint 2 --robot turtlebot4
```

#### Example 3: RoboMaster Goal-Conditioned Navigation
```bash
# Terminal 1 (keep running)
python pd_controller.py --control apf --robot robomaster

# Terminal 2
python explore_care.py --waypoint 2 --robot robomaster
```

### Parameters
- `--waypoint`: Number of waypoints for exploration (default: 2)
- `--robot`: Robot platform (`locobot`, `robomaster`, `turtlebot4`)
- `--control`: Control method for PD controller (always use `apf`)

## 🏗️ Architecture

CARE uses a two-process architecture for robust navigation:

### Process 1: PD Controller with APF
- Receives waypoints from the navigation module
- Applies Artificial Potential Field (APF) for local obstacle avoidance
- Directly controls robot motors with collision-free commands

### Process 2: Navigation Module (explore.py or explore_care.py)
- Generates waypoints using vision-based models (ViNT/NoMaD)
- CARE-enhanced version adds depth-based trajectory adjustment
- Sends waypoints to PD controller for execution

### CARE Pipeline
```
RGB Image → Depth Estimation → Top-down Map → Repulsive Forces → Adjusted Trajectory
                                                                            ↓
Robot ← PD Controller (APF) ← Waypoints ←────────────────────────────────
```

## 📝 Citation

If you find this work useful, please cite our paper:

```bibtex
@inproceedings{care2025,
  title={CARE: Enhancing Safety of Visual Navigation through Collision Avoidance via Repulsive Estimation},
  author={Kim, Joonkyung and Sim, Joonyeol and Kim, Woojun and Sycara, Katia and Nam, Changjoo},
  booktitle={Conference on Robot Learning (CoRL)},
  year={2025}
}
```

**Project Page**: https://airlab-sogang.github.io/CARE/