# Robot Navigation Project

This project implements robot navigation using Artificial Potential Field (APF) control for the LoCoBot robot.

## Quick Start

To run this project, you need to execute the following two commands:

### 1. Run the exploration script
```bash
python deployment/src/explore_apf.py --waypoint 2 --robot locobot
```

### 2. Run the PD controller
```bash
python pd_controller.py --control apf --robot locobot
```

## Prerequisites

- Python 3.x
- Required dependencies (see requirements.txt)
- LoCoBot robot setup

## Installation

1. Clone this repository
2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

The project consists of two main components:

- **Exploration Module**: Handles waypoint navigation and exploration using APF
- **PD Controller**: Provides control mechanisms using APF for the LoCoBot

Make sure to run both scripts as shown in the Quick Start section above.

## Parameters

- `--waypoint`: Number of waypoints for exploration (default: 2)
- `--robot`: Robot type (use 'locobot' for LoCoBot)
- `--control`: Control method (use 'apf' for Artificial Potential Field)

## License

[Add your license information here]

## Contributing

[Add contributing guidelines here]