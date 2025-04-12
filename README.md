#  Multi-Robot Coordination and Path Planning

### Developed by Nithish Ravikkumar & Prakul Balaji  
*Under the guidance of Dr. Felix Orlando*

---

## Overview

This project explores advanced strategies for **multi-robot path planning and coordination** using four distinct methods. The objective is to simulate and evaluate these methods for performance, collision avoidance, and scalability in complex environments.

The project focuses on the **Coordination** and **Perception** domains of Multi-Robot Systems (MRS).

---

## Key Features

- **Overhead Camera-based Multi-Robot Planning**
- **Ultrasound SLAM-based Localization and Navigation**
- **Clustering + CL-CBS for Conflict-Free Pathfinding**
- **Agglomerative Hierarchical Clustering for Optimization**
- **Animated Simulation and Visualization of Robot Movements**

---

## Methods Implemented

### Method 1: Overhead Camera-Based Coordination
- Image analysis using computer vision (OpenCV).
- Detection of robots, obstacles, targets via color segmentation and ArUco markers.
- Conflict-Based Search (CBS) combined with A* pathfinding.
- Real hardware implementation with **Mecanum Wheel Robots** and PID control.

### Method 2: Ultrasound SLAM-Based Coordination
- Real-time mapping using ultrasound sensors.
- Simultaneous localization and mapping (SLAM) for unknown environments.
- Distance sensing and map generation from scratch.
- Path planning and autonomous navigation.

### Method 3: Clustering + CL-CBS
- Clusters particles into groups and assigns robots.
- Uses the CL-CBS (Car-Like Conflict-Based Search) algorithm for movement.
- Visualized robot movement and collision-free scheduling.

### Method 4: Agglomerative Hierarchical Clustering + Heuristic Path Planning
- Simplifies paths using clustering of waypoints.
- Plans motion using heuristics and optimizes paths for minimal travel distance.
- Final approach ensures accurate goal attainment.
- Includes full-scale **matplotlib-based animation** of movements and object removals.

---

## Visualization

All algorithms are supported with clear and intuitive **matplotlib animations** showing:

- Robot trajectories
- Object collection and removal
- Collision detection and avoidance

---

## Technologies Used

- Python (OpenCV, NumPy, Matplotlib, Scikit-learn)
- C++ for CL-CBS integration
- Arduino for hardware control
- YAML for configuration and data sharing

---

## Algorithms & Concepts

- Conflict-Based Search (CBS)
- A* Search
- SLAM (Simultaneous Localization and Mapping)
- PID Control (Proportional–Integral–Derivative)
- Agglomerative Hierarchical Clustering
- Heuristic Optimization

---

## Getting Started

### Dependencies
```bash
pip install numpy opencv-python matplotlib scikit-learn pyyaml
```
## Hardware Control (Method 1)

Arduino code for robot control is provided for the **Mecanum wheel implementation**.  
Make sure to upload it to your Arduino with the correct **pin configurations** and **PID tuning parameters**.

---

## Conclusion

Among the four methods explored:

- **Method 1** excels in structured environments using an overhead perspective.
- **Method 4** offers the most adaptable and robust approach for real-world deployment.
- **Methods 2 and 3** show promise but require further optimization and integration refinement.

---

## References

- [Multi-Agent Path Planning (Python)](https://github.com/atb033/multi_agent_path_planning)
- [CL-CBS by APRIL-ZJU](https://github.com/APRIL-ZJU/CL-CBS)
- [Ultrasonic-SLAM](https://github.com/PatelVatsalB21/Ultrasonic-SLAM)
- [CBS Algorithm Paper (AAAI 2012)](https://people.engr.tamu.edu/guni/Papers/CBS-AAAI12.pdf)
- [SLAM Tutorial - Durrant-Whyte & Bailey](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf)

---

## License

This project is licensed under the **MIT License**.  
See the [LICENSE](LICENSE) file for more information.
