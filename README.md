# Multi-Robot Firefighting System

This repository contains the implementation of a **multi-robot firefighting system** designed to extinguish fires in forest environments. The project leverages **Artificial Potential Field (APF)** controllers for navigation and coordination, ensuring collision-free operation and energy-aware task allocation among robots.

---

## Features

- **Navigation**: Robots navigate through a forest environment using APF, avoiding obstacles while being attracted to fire zones.
- **Task Allocation**: Dynamic task allocation based on battery levels and proximity to fires.
- **Energy Management**: Robots monitor battery levels and return to a charging station when energy falls below a threshold.
- **Simulations**: Numerical simulations validate the system's efficiency and scalability.
- **Barrier Certificates**: Ensure collision-free movement using Control Barrier Functions (CBFs).

---

## System Architecture

1. **Robots**:
   - Number: 5
   - Initialized at random positions.
   - Capabilities:
     - Extinguishing fires.
     - Avoiding obstacles.
     - Returning to a charging station when energy is low.

2. **Environment**:
   - **Obstacles**: Represented as circular zones (e.g., unburnt trees).
   - **Fires**: Fixed locations with decreasing intensity as robots work on extinguishing them.
   - **Charging Station**: A designated area where robots recharge.

3. **Control Law**:
   - Attractive forces guide robots toward goals (fires or charging station).
   - Repulsive forces avoid collisions with obstacles and other robots.

---

## Mathematical Model

- **Potential Functions**:
  - **Attractive**: Pulls robots toward targets (fires or charging station).
  - **Repulsive**: Pushes robots away from obstacles and hazardous zones.

- **Battery Management**:
  - Battery decay rate: 0.025% per step.
  - Robots recharge upon reaching the charging station.

- **Collision Avoidance**:
  - Achieved through APF and Control Barrier Certificates.

---

## Installation

### Prerequisites
Ensure you have the following installed:
- **Python 3.8+**
- Libraries:
  - `numpy`
  - `matplotlib`
  - `rps.robotarium`
  - `os`

### Installation Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/pranaypalem/artificial-potential-field.git
   cd artificial-potential-field
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. (Optional) Set up the environment for saving simulation frames:
   Update the `frame_dir` variable in the Python script to your preferred directory for saving simulation frames.

4. Download the fire image for visualization:
   Place an image named `fire.png` in the root directory of the project for simulation visualization.

---

## Usage

### Running the Simulation

To start the simulation:

```bash
python Final_code.py
```

### Expected Outputs
- **Simulation Frames**: Captures of the robot movements.
- **Trajectory Plots**: Robots navigating toward fires while avoiding obstacles.
- **Battery Annotations**: Dynamic updates of battery levels over time.
- **Fire Status Updates**: Visualization of fire intensity reduction as robots work to extinguish them.

---

## Key Functions

- **attractive_force**:
  Computes the attractive gradient pulling robots toward the goal.

- **repulsive_force**:
  Calculates the repulsive gradient for collision avoidance.

- **within_fire_zone**:
  Determines if a robot is within the radius of a fire zone.

---

## Robotarium Simulation

The Robotarium platform was used to simulate and test the multi-robot system in a controlled environment. Key features of Robotarium usage include:

- **Dynamic Testing**: Verified collision-free navigation using barrier certificates.
- **Visualization**: Enabled real-time observation of robot paths and behaviors.
- **Code Validation**: Ensured the APF-based navigation and energy-aware strategies worked as intended in a physical-like simulation setup.

---

## Simulation Results

1. **Navigation**:
   - Robots successfully reached fire zones while avoiding obstacles.
2. **Energy Management**:
   - Robots returned to the charging station when battery levels dropped below 25%.
3. **Task Allocation**:
   - Fires were extinguished sequentially, optimizing energy usage and efficiency.

---

## Future Work

- **Local Minima Handling**: Improve APF to avoid trapping robots.
- **Dynamic Task Allocation**: Integrate learning-based methods for enhanced coordination.
- **Real-World Deployment**: Extend simulations to physical robots.

