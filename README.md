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

This creates the executable:

```
build/teeth_exec
```

---

## ▶️ Run

Make sure the xArm ROS driver is **not** running.

```
./teeth_exec
```

Robot IP is set inside `src/main.cpp`:

```cpp
XArmAPI arm("192.168.1.221");
```

Change it to your robot’s IP.

---

