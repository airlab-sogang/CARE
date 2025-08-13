# Robot Navigation Project

This project implements robot navigation using Artificial Potential Field (APF) control for multiple robot platforms.

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

- Python 3.x
- Required dependencies (see requirements.txt)
- Robot hardware setup for chosen platform

## Installation

1. Clone this repository
2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

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

## License

[Add your license information here]

## Contributing

[Add contributing guidelines here]