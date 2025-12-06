# Teeth Controller (xArm C++)

A small pure C++ project for controlling the xArm robot using the official **xArm C++ SDK**.

---

## 📁 Project Structure

```
teeth_controller/
├── CMakeLists.txt
├── src/
│   └── main.cpp
└── thirdparty/
    └── xArm-CPLUS-SDK/
```

---

## 🔧 Requirements
- CMake ≥ 3.8  
- g++ or clang  
- pthread  
- xArm robot reachable by IP  

---

## 📥 Install (with Git Submodule)

If you want to include the xArm C++ SDK using a **git submodule**:

### 1. Clone your project
```
git clone <your_repo_url>
cd teeth_controller
```

### 2. Initialize the submodule
```
git submodule update --init --recursive
```

If the SDK is not yet added as a submodule, add it manually:

```
git submodule add https://github.com/xArm-Developer/xArm-CPLUS-SDK.git thirdparty/xArm-CPLUS-SDK
git submodule update --init --recursive
```

To update the SDK later:

```
git submodule update --remote --merge
```

---

## 🚀 Build Instructions

```
mkdir build
cd build
cmake ..
make -j4
```

This creates the executables files



# 🦷 DentalRobot — Operation & Alignment Guide

This document explains how to run the DentalRobot system, including launching the GUI, running the robot driver, starting the RealSense camera, performing alignment, and executing inner/outer cleaning routines.

---

# 📁 1. GUI Operation

The GUI allows you to upload the teeth model and manually select the region to clean.

### **Launch the GUI**
```bash
cd /home/amrish/Documents/DentalRobot/DentalRepo
conda activate /home/amrish/miniconda3/envs/dental_gui
python3 main.py
```

---

# 🤖 2. Robot Driver (xArm + MoveIt)

Start the xArm robot driver with MoveIt.  
RealSense D435i integration is enabled using the launch argument.

```bash
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py     robot_ip:=192.168.1.221 add_realsense_d435i:=true
```

---

# 📷 3. RealSense Camera Node

Launch the RealSense D435i camera with correct calibration/configuration files.

```bash
ros2 launch realsense2_camera rs_launch.py     config_file:=/home/amrish/realsense_params/d435if_ros.yaml     json_file_path:=/home/amrish/realsense_params/d435if_optimal.json
```

---

# 🎯 4. Alignment Process

The alignment node takes teeth waypoints in the **teeth frame** and outputs transformed waypoints in the **robot base frame**.

### **Input:**  
`/tmp/waypoints_teeth.csv`

### **Output:**  
`/tmp/waypoints_dummy_in_base.csv`

### **Run the aligner:**
```bash
ros2 run jaw_alignment teeth_aligner <path/to/file.ply>
```

---

# 🛠️ 5. Cleaning Execution (After Alignment)

After alignment completes, use the transformed CSV file to execute the cleaning routines.

---

## 🧼 Outer Surface Cleaning

```bash
cd /home/amrish/dental_ws/src/teeth_controller-/build
./outer_cleaner /tmp/waypoints_dummy_in_base.csv
```

---

## 🧼 Inner Surface Cleaning

```bash
cd /home/amrish/dental_ws/src/teeth_controller-/build
./inner_cleaner /tmp/waypoints_dummy_in_base.csv
```

---

# ✔ Notes

- Ensure the GUI export generates `/tmp/waypoints_teeth.csv` before running the aligner.
- Confirm that the robot, camera, and GUI are all running before executing cleaning programs.
- Alignment must be performed **every time the mouth or jaw position changes**.

---

