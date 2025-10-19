```markdown
# 🛰️ ANVESHAK – LunaBot Autonomous Rover System

**Smart India Hackathon 2025 – Problem Statement ID: 25169**  
**Theme:** Space Technology | **Team Name:** SKYVAR  

---

## 🚀 Overview  
**ANVESHAK (LunaBot)** is an end-to-end autonomous lunar exploration system that simulates and controls a rover capable of navigating both **indoor habitats** and **rugged lunar terrain** — without GPS.  
It integrates **Unity 3D simulation**, **ROS2-based autonomy**, and an interactive **LunaCommand Dashboard** for real-time rover control and monitoring.

---

## 🧠 System Architecture  

### **Layer 0 – Terrain Data**
- Uses **ISRO DEM** and **NASA LRO datasets** as the lunar terrain base.

### **Layer 1 – Simulation**
- Built in **Unity 3D (HDRP)** using **C#**.
- Simulates physics, LiDAR, and camera sensors.

### **Layer 2 – ROS2 Navigation**
- Built with **ROS2 Humble**, **Python**, and **Nav2 Stack**.
- Handles path planning, SLAM, and inter-node communication.

### **Layer 3 – AI & Vision**
- Uses **PyTorch**, **OpenCV**, and **CNN/RL models**.
- Performs **dust filtering**, **terrain segmentation**, and **adaptive AI planning**.

### **Layer 4 – LunaCommand Dashboard**
- Web UI built with **React.js**, **Three.js**, and **Tailwind CSS**.
- Displays live 3D rover map, control panels, and telemetry.

### **Layer 5 – Backend Bridge**
- **Node.js + WebSocket + ROS Bridge** enable real-time communication between the rover, simulation, and dashboard.

---

## ⚙️ Key Features  
- **Improved Lunar Autonomy:** GPS-free navigation for indoor and outdoor terrains.  
- **Dust-Tolerant Vision:** AI filter cleans camera & LiDAR input.  
- **Integrated SLAM:** Combines LiDAR, IMU, and camera for precise localization.  
- **Real-Time Command Center:** Dashboard for mission planning and rover control.  
- **Dual-Environment Adaptability:** Switches between indoor structured mapping and outdoor rugged terrain.  
- **Predictive Maintenance Node:** ML-based detection of motor, wheel, and power anomalies.  
- **Energy-Efficient Navigation:** Optimized route planning to conserve power.  
- **Multi-Rover Collaboration:** Real-time data sharing for swarm operation.

---

## 🧩 Repository Structure  
```

ANVESHAK/
├── LUNA_COMMAND_DASHBOARD/   # Web UI for mission control
├── Simulation/                # Unity 3D lunar environment
├── ros2_ws/                   # ROS2 workspace for navigation & AI nodes
│   └── src/
└── README.md

````

---

## 🖥️ How to Run  

### 1. Clone the Repo
```bash
git clone https://github.com/YUVAN0907/ANVESHAK.git
cd ANVESHAK
````

### 2. Run the Dashboard

```bash
cd LUNA_COMMAND_DASHBOARD
npm install
npm run start
```

### 3. Launch ROS2

```bash
cd ../ros2_ws
colcon build
source install/setup.bash
ros2 launch anveshak main.launch.py
```

### 4. Launch Simulation

Open the `Simulation` folder in **Unity 3D (HDRP)** and run the lunar environment.

---

## 🌕 Outcome

ANVESHAK demonstrates a **complete autonomous lunar mission simulation**, integrating:

* AI-aided navigation
* Multi-sensor fusion SLAM
* Real-time mission command dashboard
* Predictive maintenance and multi-rover collaboration


---

```

