# Autonomous Search and Rescue Drone using PX4, ROS 2 and Computer Vision

## Overview

This project implements an autonomous **Search and Rescue (SAR)** drone simulation using **PX4 SITL**, **ROS 2 Humble**, and **Gazebo Classic**. The drone autonomously covers a predefined search area, detects humans using computer vision, geotags their locations, clusters duplicate detections, and visualizes the results on interactive maps.

The system demonstrates an end-to-end SAR pipeline that integrates autonomous navigation, perception, localization, and visualization.

---

## Features

* Autonomous offboard flight using PX4 and ROS 2
* Lawnmower coverage path generation
* Simulated RGB camera integration
* Real-time human detection using YOLO
* Pixel-to-world coordinate projection
* Conversion from local NED coordinates to GPS coordinates
* Geotag generation and logging
* Duplicate detection removal using DBSCAN clustering
* CSV and GeoJSON export
* Folium-based interactive result visualization
* RViz visualization of:

  * Coverage path
  * Drone trajectory
  * Detection image stream

---

## System Architecture

```
PX4 SITL + Gazebo
        │
        ▼
Offboard Control Node
        │
        ▼
Camera Controller
        │
        ▼
YOLO Detection Node
        │
        ▼
/drone/detections
        │
        ▼
Geotag Node
        │
        ├── geotags.csv
        ├── geotags.geojson
        ├── clustered_geotags.csv
        └── clustered_geotags.geojson
        │
        ▼
Visualization Script
        │
        ▼
results_map.html
results_map.png
```

---

## Search Area

The drone performs autonomous coverage of a:

* Search Area: 20 m × 20 m
* Flight Altitude: 8 m
* Strip Width: 4 m
* Coverage Pattern: Lawnmower (boustrophedon)

Coverage waypoints are published on:

```
/drone/coverage_path
```

Drone trajectory is published on:

```
/drone/trajectory
```

---

## Human Detection Pipeline

### 1. Image Acquisition

The onboard camera captures RGB images from Gazebo and publishes them to the detection pipeline.

### 2. Human Detection

YOLO processes incoming frames and outputs:

```json
[
  {
    "cx": 412,
    "cy": 251,
    "confidence": 0.63
  }
]
```

Published on:

```
/drone/detections
```

### 3. Geotag Generation

For every detection:

1. Convert image pixels to camera rays
2. Transform rays into the drone body frame
3. Transform body frame into NED coordinates using vehicle attitude
4. Intersect the viewing ray with the ground plane
5. Convert NED coordinates to GPS coordinates

Generated geotag:

```json
{
  "x": 6.43,
  "y": 6.79,
  "lat": 47.39779,
  "lon": 8.54568,
  "confidence": 0.48,
  "timestamp": 1782226081.82
}
```

---

## Duplicate Detection Removal

Since the same person may be observed multiple times during flight, duplicate detections are merged using DBSCAN clustering.

Each cluster stores:

* Average position
* Average confidence
* Detection count
* Latest timestamp

Outputs:

```
clustered_geotags.csv
clustered_geotags.geojson
```

---

## Repository Structure

```
soc_ws/
│
├── src/
│   ├── sar_coverage/
│   │   └── offboard_control.py
│   │
│   ├── sar_detection/
│   │   └── detection_node.py
│   │
│   ├── sar_geotag/
│   │   └── geotag_node.py
│   │
│   └── sar_bringup/
│       ├── launch/
│       │   └── sar_full.launch.py
│       └── scripts/
│           └── visualize_results.py
│
├── geotags.csv
├── geotags.geojson
├── clustered_geotags.csv
├── clustered_geotags.geojson
├── results_map.html
└── results_map.png
```

---

## Installation

### Clone Repository

```bash
git clone <repository_url>
cd soc_ws
```

### Build Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Simulation

Launch the complete SAR pipeline:

```bash
source ~/soc_ws/install/setup.bash
ros2 launch sar_bringup sar_full.launch.py
```

---

## Generate Results

```bash
cd ~/soc_ws
source install/setup.bash
python3 ~/soc_ws/src/sar_bringup/scripts/visualize_results.py
xdg-open ~/soc_ws/results_map.html
```

---

## ROS Topics

### Navigation

```
/drone/coverage_path
/drone/trajectory
```

### Vision

```
/drone/detection_image
/drone/detection_boxes
/drone/detections
```

### Localization

```
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_attitude
```

### Geotagging

```
/drone/geotags
```

---

## Outputs

### CSV Files

* geotags.csv
* clustered_geotags.csv

### GeoJSON Files

* geotags.geojson
* clustered_geotags.geojson

### Visualizations

* results_map.html
* results_map.png

---

## Technologies Used

* ROS 2 Humble
* PX4 SITL
* Gazebo Classic
* Python
* OpenCV
* Ultralytics YOLO
* NumPy
* Scikit-learn
* Folium
* Geopy
* RViz2

---

## Future Work

* Multi-UAV cooperative search
* Adaptive coverage planning
* Survivor prioritization and ranking
* Real-time map streaming
* Integration with real PX4 hardware
* Thermal camera support
* Terrain-aware path planning

---

## Authors

Developed as an autonomous Search and Rescue simulation project integrating robotics, computer vision, and autonomous navigation using ROS 2 and PX4.
