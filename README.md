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
pip install -e diffusion_policy/ --config-settings editable_mode=compat

# Install additional packages
pip install rospkg efficientnet_pytorch warmup_scheduler diffusers vit-pytorch
pip install --upgrade "numpy<2,>=1.26"
```
Note: The following pip dependency resolver message may appear after installation. It does not affect running this project and can be safely ignored.

```
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
robomaster 0.1.1.65 requires netifaces>=0.10, which is not installed.
unidepth 0.1 requires numpy>=2.0.0, but you have numpy 1.26.4 which is incompatible.
```

## 📖 Usage

### ⚠️ Important: Two-Process Architecture
CARE requires running two processes simultaneously:
1. **PD Controller** (Terminal 1) - Handles robot control with PD controller
2. **Navigation/Exploration Module** (Terminal 2) - Provides waypoint generation

### Supported Robot Platforms
- **LoCoBot** - 170° FOV fisheye camera
- **TurtleBot4** - 89.5° FOV OAK-D Pro camera
- **RoboMaster S1** - 120° FOV camera

### Navigation Modes

#### 1. Exploration Mode
Autonomous exploration with collision avoidance:

**Terminal 1 (PD Controller):**
```bash
# Standard exploration
python deployment/src/pd_controller.py --control nomad --robot [ROBOT_TYPE]

# CARE-enhanced exploration
python deployment/src/pd_controller.py --control care --robot [ROBOT_TYPE]
```

**Terminal 2 (Choose one):**
```bash
# Standard exploration
python deployment/src/explore.py --waypoint 2 --robot [ROBOT_TYPE]

# CARE-enhanced exploration
python deployment/src/explore_care.py --waypoint 2 --robot [ROBOT_TYPE]
```

#### 2. Goal-Conditioned Navigation
Navigate using topological maps with ViNT or NoMaD models:

**Terminal 1 (PD Controller - Required):**
```bash
# Standard navigation
python deployment/src/pd_controller.py --control nomad --robot [ROBOT_TYPE]

# CARE-enhanced navigation
python deployment/src/pd_controller.py --control care --robot [ROBOT_TYPE]
```

**Terminal 2 (Choose one):**
```bash
# Standard goal-conditioned navigation
python deployment/src/navigate.py \
    --waypoint 2 \
    --dir [TOPOMAP_DIR] \
    --model [MODEL_TYPE] \
    --robot [ROBOT_TYPE]

# CARE-enhanced goal-conditioned navigation
python deployment/src/navigate_care.py \
    --waypoint 2 \
    --dir [TOPOMAP_DIR] \
    --model [MODEL_TYPE] \
    --robot [ROBOT_TYPE]
```

### Parameters

#### Common Parameters
- `--waypoint`: Number of waypoints for navigation (default: 2)
- `--robot`: Robot platform (`locobot`, `robomaster`, `turtlebot4`)

#### PD Controller Parameters
- `--control`: Control method (`nomad`, `care`)

#### Navigation-specific Parameters
- `--dir`: Directory containing the topological map (e.g., `topomap_locobot`)
- `--model`: Navigation model to use (`vint`, `nomad`)

### Complete Usage Examples

#### Example 1: LoCoBot CARE-Enhanced Exploration
```bash
# Terminal 1 (keep running)
python deployment/src/pd_controller.py --control care --robot locobot

# Terminal 2
python deployment/src/explore_care.py --waypoint 2 --robot locobot
```

#### Example 2: TurtleBot4 Standard Exploration
```bash
# Terminal 1 (keep running)
python deployment/src/pd_controller.py --control nomad --robot turtlebot4

# Terminal 2
python deployment/src/explore.py --waypoint 2 --robot turtlebot4
```

#### Example 3: LoCoBot Goal-Conditioned Navigation with ViNT (Standard)
```bash
# Terminal 1 (keep running)
python deployment/src/pd_controller.py --control nomad --robot locobot

# Terminal 2
python deployment/src/navigate.py \
    --waypoint 2 \
    --dir topomap_locobot \
    --model vint \
    --robot locobot
```

#### Example 4: LoCoBot Goal-Conditioned Navigation with ViNT (CARE-Enhanced)
```bash
# Terminal 1 (keep running)
python deployment/src/pd_controller.py --control care --robot locobot

# Terminal 2
python deployment/src/navigate_care.py \
    --waypoint 2 \
    --dir topomap_locobot \
    --model vint \
    --robot locobot
```

#### Example 5: RoboMaster Goal-Conditioned Navigation with NoMaD (CARE-Enhanced)
```bash
# Terminal 1 (keep running)
python deployment/src/pd_controller.py --control care --robot robomaster

# Terminal 2
python deployment/src/navigate_care.py \
    --waypoint 2 \
    --dir topomap_robomaster \
    --model nomad \
    --robot robomaster
```

#### Example 6: TurtleBot4 Goal-Conditioned Navigation with NoMaD (Standard)
```bash
# Terminal 1 (keep running)
python deployment/src/pd_controller.py --control nomad --robot turtlebot4

# Terminal 2
python deployment/src/navigate.py \
    --waypoint 2 \
    --dir topomap_turtlebot4 \
    --model nomad \
    --robot turtlebot4
```

## 🏗️ Architecture

CARE uses a two-process architecture for robust navigation:

### Process 1: PD Controller
- Receives waypoints from the navigation/exploration module
- Directly controls robot motors with collision-free commands
- Can operate in standard (`nomad`) or CARE-enhanced (`care`) mode

### Process 2: Navigation/Exploration Module
- **Exploration** (`explore.py` / `explore_care.py`): Generates waypoints for autonomous exploration
- **Goal-Conditioned Navigation** (`navigate.py` / `navigate_care.py`): Uses topological maps and vision models (ViNT/NoMaD) for targeted navigation
- CARE-enhanced versions add depth-based trajectory adjustment
- Sends waypoints to PD controller for execution

### CARE Pipeline
```
RGB Image → Depth Estimation → Top-down Map → Repulsive Forces → Adjusted Trajectory
                                                                            ↓
Robot ← PD Controller ← Waypoints ←────────────────────────────────
```

### Navigation Models
- **ViNT**: Visual Navigation Transformer for goal-conditioned navigation
- **NoMaD**: No Maps, No Problem - Exploration and navigation without explicit mapping

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