# Robotic Arm Inverse Kinematics — Analytical vs Machine Learning

## Overview

This project implements and compares two approaches for solving the inverse kinematics (IK) of a 3-DOF robotic arm:

1. **Analytical IK Solver** (C++)
2. **Machine Learning IK Solver** (PyTorch + ONNX + C++ inference)

Both approaches are integrated into a full motion planning pipeline with:

* Forward Kinematics
* Collision Detection
* RRT-Connect Trajectory Planning
* Obstacle Avoidance
* Visualisation
* Performance Benchmarking

The ML model is trained using a large synthetic dataset generated from the robot's forward kinematics.

---

# Robot Configuration

3-DOF revolute robotic arm:

| Link | Length (meters) |
| ---- | --------------- |
| L1   | 0.30            |
| L2   | 0.25            |
| L3   | 0.15            |

Workspace: 3D Cartesian space

---

# Project Structure

```
ML_Assignment/
│
├── include/                  # Header files
│   ├── RobotArm.h
│   ├── IKSolver.h
│   ├── NeuralIKSolver.h
│   └── ...
│
├── src/                      # C++ implementation
│   ├── RobotArm.cpp
│   ├── IKSolver.cpp
│   ├── NeuralIKSolver.cpp
│   ├── Planner.cpp
│   └── ...
│
├── apps/
│   └── main.cpp              # Main comparison application
│
├── ML/                       # Machine Learning components
│   ├── train_fk_model.py
│   ├── neural_ik_solver.py
│   ├── fk_model.pth
│   ├── fk_model.onnx
│   ├── scaler_angles.save
│   ├── scaler_pos.save
│   └── benchmark_ik.py
│
├── external/onnxruntime/    # ONNX Runtime
│
├── visualiser_planner.py    # 3D visualization
│
├── results/
│   └── ik_comparison.png
│
├── CMakeLists.txt
└── README.md
```

---

# Features

## Analytical IK (C++)

Uses geometric closed-form solution:

* Cosine law
* Exact mathematical solution
* No training required
* Extremely fast (~microseconds)

File:

```
src/IKSolver.cpp
```

---

## Machine Learning IK

Neural network learns forward kinematics and solves IK using gradient optimization.

Architecture:

```
Input: 3 (angles)
Hidden: 256
Hidden: 256
Hidden: 256
Output: 3 (position)
Activation: GELU
```

Files:

```
ML/train_fk_model.py
ML/neural_ik_solver.py
```

---

## C++ ML Integration

The trained model is exported to ONNX and used in C++ with ONNX Runtime.

```
src/NeuralIKSolver.cpp
```

---

## Trajectory Planning

Uses RRT-Connect algorithm with collision checking.

Features:

* obstacle avoidance
* multiple planning attempts
* safe trajectory selection

Files:

```
src/Planner.cpp
src/CollisionChecker.cpp
```

---

## Visualization

3D real-time animation using matplotlib.

```
visualiser_planner.py
```

Displays:

* robot motion
* target
* obstacle
* trajectory attempts
* collision points

---

# Installation

## Requirements

### Windows

Install:

* Python 3.11
* CMake
* MinGW64
* PyTorch
* ONNX Runtime

Python packages:

```
pip install torch numpy matplotlib joblib onnxruntime
```

---

# Build Instructions

From project root:

```
mkdir build
cd build
cmake ..
cmake --build .
```

This creates:

```
main_app.exe
```

---

# Running the Project

Run C++ planner:

```
build\main_app.exe
```

This generates:

```
trajectory_safe_analytic.txt
trajectory_safe_neural.txt
scene.txt
```

---

# Visualize Trajectories

From project root:

```
python visualiser_planner.py
```

---

# Machine Learning Training

Train model:

```
cd ML
python train_fk_model.py
```

This creates:

```
fk_model.pth
scaler_angles.save
scaler_pos.save
```

---

# Benchmark and Comparison

Run benchmark:

```
python ML/benchmark_ik.py
```

Generates:

```
results/accuracy.png
results/speed.png
results/success_rate.png
```

---

# Performance Comparison

| Metric         | Analytical IK  | ML IK           |
| -------------- | -------------- | --------------- |
| Accuracy       | Perfect        | Near-perfect    |
| Speed          | Extremely fast | Slightly slower |
| Memory         | Minimal        | Higher          |
| Training       | None           | Required        |
| Generalization | Limited        | High            |

---

# Example Output

Analytical IK:

```
0.620249 0.0127932 0.879764
```

ML IK:

```
0.620248 0.0127935 0.879763
```

Position error:

```
< 0.000001 meters
```

---

# Advantages of Analytical IK

* exact solution
* instant computation
* no training required
* minimal memory

---

# Advantages of ML IK

* works for complex robots
* adaptable
* generalizes well
* useful when analytical solution unavailable

---

# Future Improvements

* GPU ONNX inference
* transformer-based IK
* real robot deployment
* reinforcement learning integration

---

# Author

Aarish Patel
Machine Learning Engineer Intern Technical Assessment

GitHub:
https://github.com/Aarish-Patel/ML_Assignment

---

# License

This project is for educational and assessment purposes.
