# Robot Navigation Project

This repository contains the implementation of CARE: Enhancing Safety of Visual Navigation through Collision Avoidance via Repulsive Estimation, accepted to CoRL 2025.

## Environment Setup

### 1. Create Conda Environment

```bash
# Create conda environment with Python 3.10
conda create -n CARE python=3.10
conda activate CARE
```

### 2. Clone and Install Core Dependencies

```bash
# Clone and install UniDepth
git clone https://github.com/lpiccinelli-eth/UniDepth.git
pip install -e UniDepth/ --extra-index-url https://download.pytorch.org/whl/cu118

# Clone and install visualnav-transformer
git clone https://github.com/robodhruv/visualnav-transformer.git
pip install -e train/

# Clone and install diffusion_policy
git clone https://github.com/real-stanford/diffusion_policy.git
pip install -e diffusion_policy/
```

### 3. Install Additional Dependencies

```bash
# Install Python packages
pip install rospkg
pip install efficientnet_pytorch warmup_scheduler diffusers
pip install vit-pytorch

# Install conda packages
conda install conda-forge::opencv -y
conda install conda-forge::einops -y
conda install conda-forge::wandb -y
conda install conda-forge::prettytable -y

# Install numpy with version constraint
conda install "numpy<2,>=1.26" -y
```

### 4. Fix Diffusion Policy Installation (if needed)

If you encounter issues with diffusion_policy, ensure the package structure is correct:

```bash
# Verify the structure exists:
# visualnav-transformer-test/
#   diffusion_policy/
#     setup.py or pyproject.toml
#     diffusion_policy/
#       __init__.py        <-- Should exist
#       model/
#         diffusion/
#           conditional_unet1d.py  <-- Should exist

# Reinstall if needed
pip install -e diffusion_policy/
```

## Quick Start

This project provides two main exploration modes:

### Standard Exploration
```bash
python explore.py --waypoint 2 --robot [ROBOT_TYPE]
```

### Care-Enhanced Exploration
```bash
python explore_care.py --waypoint 2 --robot [ROBOT_TYPE]
```

### PD Controller (Optional)
```bash
python pd_controller.py --control apf --robot [ROBOT_TYPE]
```

## Supported Robots

The project supports three robot platforms:
- **locobot** - LoCoBot robot platform
- **robomaster** - DJI RoboMaster robot
- **turtlebot4** - TurtleBot 4 robot platform

## Usage Examples

### Standard Exploration Examples
```bash
# LoCoBot exploration
python explore.py --waypoint 2 --robot locobot

# RoboMaster exploration  
python explore.py --waypoint 2 --robot robomaster

# TurtleBot4 exploration
python explore.py --waypoint 2 --robot turtlebot4
```

### Care-Enhanced Exploration Examples
```bash
# LoCoBot with care features
python explore_care.py --waypoint 2 --robot locobot

# RoboMaster with care features
python explore_care.py --waypoint 2 --robot robomaster

# TurtleBot4 with care features
python explore_care.py --waypoint 2 --robot turtlebot4
```

## Exploration Modes

- **explore.py**: Standard navigation and exploration using APF
- **explore_care.py**: Enhanced exploration with additional care features (collision avoidance, safety protocols, etc.)

## Prerequisites

- Python 3.10
- Conda package manager
- CUDA 11.8 compatible GPU (for PyTorch installation)
- Robot hardware setup for chosen platform

## Installation Summary

1. Set up conda environment (see Environment Setup section above)
2. Clone this repository
3. Ensure all dependencies are installed following the Environment Setup steps

## Parameters

- `--waypoint`: Number of waypoints for exploration (default: 2)
- `--robot`: Robot type (`locobot`, `robomaster`, or `turtlebot4`)
- `--control`: Control method (use 'apf' for Artificial Potential Field)

## Project Structure

The project consists of main components:

- **Exploration Modules**:
    - `explore.py` - Standard waypoint navigation using APF
    - `explore_care.py` - Enhanced navigation with care features
- **PD Controller**: Provides control mechanisms using APF for various robots

## Troubleshooting

### Common Issues

1. **PyTorch CUDA Issues**: Ensure CUDA 11.8 is installed and compatible with your GPU
2. **Diffusion Policy Import Errors**: Verify the package structure and `__init__.py` files exist
3. **NumPy Version Conflicts**: Use the specified numpy version constraint: `"numpy<2,>=1.26"`

### Environment Verification

To verify your environment is set up correctly:

```bash
conda activate CARE
python -c "import torch; print(f'PyTorch version: {torch.__version__}')"
python -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"
python -c "import numpy; print(f'NumPy version: {numpy.__version__}')"
```

## License

[Add your license information here]

## Contributing

[Add contributing guidelines here]
