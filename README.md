# MPPI Vehicle Parking Simulator

This repository contains a simple C++ implementation of a Model Predictive Path Integral (MPPI) controller
for a bicycle‐model vehicle. The simulation performs a parking maneuver while avoiding static obstacles. A
Python script is provided for plotting results and another for tuning controller parameters.

## Contents

- `main.cpp` – top‑level program running the simulation
- `model.h` / `model.cpp` – vehicle dynamics (bicycle model with lateral force)
- `MPPIController.h` / `MPPIController.cpp` – MPPI algorithm and cost functions
- `mppi.cfg` – plain text configuration for weights, horizon, noise, etc.
- `plot.py` – generate plots from `states.csv` and `obstacles.csv`
- `tuner.py` – random‑search tuning script that modifies `mppi.cfg`

## Build and run

This project uses Eigen for vector/matrix operations. On Windows the easiest way
is to bootstrap `vcpkg` and install Eigen:

```powershell
cd <repo_root>
# assuming vcpkg subdirectory already present
vcpkg\bootstrap-vcpkg.bat
vcpkg\vcpkg.exe install eigen3:x64-windows
```

Compile with g++ (adjust include path as needed):

```powershell
g++ -std=c++17 main.cpp model.cpp MPPIController.cpp \
    -I./vcpkg/installed/x64-windows/include/eigen3 \
    -o vehicle_sim.exe
```

Run the simulator:

```powershell
./vehicle_sim.exe
```

It will produce `states.csv` (state trajectory) and `obstacles.csv` (static
vehicles). The console also prints tabular data and total cost.

## Configuration

Edit `mppi.cfg` to change parameters such as the number of samples/horizon,
noise covariance, cost weights, obstacle buffer (`obstacle_eps`), and control
limits. The simulator reads this file at startup; missing values fall back to
defaults hard‑coded in `MPPIController`.

Example entries:

```
K 1000
N 30
lambda 1.0
Sigma00 0.05
Sigma11 100.0
pos_cost_weight 200.0
vel_cost_weight 100.0
delta_cost_weight 50.0
collision_cost_weight 1e1
terminal_cost_weight 1000.0
terminal_zone_radius 0.2
obstacle_eps 0.5
```

## Plotting results

Once `states.csv` exists, visualize with:

```bash
python plot.py
```

This opens a multi‑panel figure showing trajectory, velocities, yaw, slip angle,
yaw rate, steering angle and longitudinal force. Oriented rectangles mark
obstacles.

## Parameter tuning

Run `tuner.py` to search randomly for low‑cost weight combinations. It will
overwrite `mppi.cfg` for each trial and copy the best result to `best.cfg`.
Adjust `param_ranges` inside the script to change the search bounds or add new
parameters.

```bash
python tuner.py
```

Feel free to fork, experiment with different costs or dynamics, and push your
changes back to GitHub. Happy parking!