# UAV-attack-simulation
# 🚁 Drone Multilateration over Ad-Hoc WiFi — ns-3 Simulation

A network simulator (ns-3) project that demonstrates **real-time drone position estimation** using multilateration over an ad-hoc WiFi network. Drones estimate their own 2D position by receiving beacon messages from anchor nodes (trucks, other drones, or both) and solving a system of distance equations.

## 📌 Overview

A convoy of trucks moves linearly along the X axis. One or more drones fly nearby at a configurable altitude and velocity. Each anchor node periodically broadcasts its known position via UDP over 802.11a (5 GHz). Drones collect these beacons, compute planar distances (compensating for altitude via 3D→2D projection), and apply **multilateration** to estimate their own position.

The simulation compares estimated positions against ground truth and reports accuracy metrics, including automatic **spoofing detection** when the estimation error exceeds a threshold.

## ✨ Features

- **Hybrid anchor system** — trucks, drones, or both can act as anchors (`--anchorMode`)
- **2-anchor mode** — circle intersection fallback when only 2 anchors are available
- **3+ anchor trilateration** — selects the most geometrically diverse anchor triplet (anti-collinearity)
- **Least-squares fallback** — handles degenerate/collinear anchor configurations
- **3D → 2D projection** — compensates altitude differences for accurate planar distance
- **Spoofing detection** — alerts when multilateration error exceeds a configurable threshold
- **Batch testing** — `run_all_tests.sh` runs all parameter combinations and exports results to CSV
- **Fully configurable** — number of drones, trucks, speeds, and anchor mode via command-line arguments

## 🛠 Requirements

- [ns-3.46](https://www.nsnam.org/) (Network Simulator 3)
- C++17 compiler (GCC 9+ recommended)
- Linux / WSL

## 🚀 Quick Start

```bash
# 1. Copy the simulation file into ns-3 scratch directory
cp scratch/DronesFinal.cc <ns-3.46-path>/scratch/

# 2. Build
cd <ns-3.46-path>
./ns3 configure --enable-examples
./ns3 build

# 3. Run a single simulation
./ns3 run "DronesFinal --nDrones=2 --nTrucks=5 --anchorMode=trucks"

# 4. Run all parameter combinations (batch)
chmod +x run_all_tests.sh
./run_all_tests.sh
```

## ⚙️ Command-Line Parameters

| Parameter | Default | Description |
|---|---|---|
| `--nDrones` | 2 | Number of drones |
| `--nTrucks` | 10 | Number of trucks (anchor vehicles) |
| `--droneSpeedX` | 5.0 | Drone velocity along X axis (m/s) |
| `--droneSpeedY` | 0.0 | Drone velocity along Y axis (m/s) |
| `--droneSpeedZ` | 0.0 | Drone velocity along Z axis (m/s) |
| `--droneStartX` | 50.0 | Drone starting X position (m) |
| `--droneStartY` | 50.0 | Drone starting Y position (m) |
| `--droneStartZ` | 20.0 | Drone altitude (m) |
| `--truckSpeed` | 5.0 | Truck convoy speed along X (m/s) |
| `--anchorMode` | hybrid | Anchor mode: `trucks`, `drones`, or `hybrid` |

## 📊 Anchor Modes

| Mode | Who sends beacons | Min. nodes needed |
|---|---|---|
| `trucks` | Only trucks | ≥ 2 trucks |
| `drones` | Only drones | ≥ 3 drones (each sees N-1 anchors) |
| `hybrid` | Both trucks and drones | ≥ 2 total anchor nodes |

## 📈 Sample Output

```
[t=1.1s] Drone 10  |  ESTIMATED: (48.2, 51.3)  |  REAL: (50.5, 50.0)  |  ERROR: 2.61 m

========================================
SIMULATION RESULTS
  Anchor mode:        trucks
  Total measurements: 2314
  Average error:      5.91 m
========================================
```

## 📂 Project Structure

```
├── scratch/
│   └── DronesFinal.cc      # Main simulation source code
├── run_all_tests.sh         # Batch runner for all parameter combinations
└── README.md
```

## 📝 License

This project is for academic and research purposes.
