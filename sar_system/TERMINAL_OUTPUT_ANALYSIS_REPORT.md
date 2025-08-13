# SAR System Terminal Output Analysis Report

## Executive Summary

This report analyzes the terminal output from both SAR (Search and Rescue) systems launched on ROS2 Jazzy with Gazebo Harmonic. The systems were launched in sequence and monitored for approximately 15 seconds to identify potential issues and provide solutions.

**Systems Tested:**
- Go2 SAR System (Unitree Go2 quadruped robot)
- Drone SAR System (PX4-based drone with MAVROS)

**Test Duration:** 15 seconds
**Environment:** ROS2 Jazzy + Gazebo Harmonic
**Date:** August 13, 2025

---

## Go2 SAR System Analysis

### ✅ **Successful Components**

| Component | Status | Description |
|-----------|--------|-------------|
| Gazebo Simulation | ✅ **SUCCESS** | Gazebo world launched successfully |
| Parameter Bridge | ✅ **SUCCESS** | All sensor bridges created and initialized |
| Robot State Publisher | ✅ **SUCCESS** | Robot initialized and publishing transforms |
| Controller Manager | ✅ **SUCCESS** | All hardware interfaces configured and activated |
| Sensors | ✅ **SUCCESS** | IMU, LiDAR, and camera sensors operational |
| RViz2 | ✅ **SUCCESS** | Visualization launched (with warnings) |

### ⚠️ **Issues Identified**

#### 1. **TF2 Time Jump Warnings** - 🔴 **CRITICAL**
```
[ekf_node-6] [WARN] Detected jump back in time. Clearing TF buffer.
[ekf_node-7] [WARN] Detected jump back in time. Clearing TF buffer.
[rviz2-8] [WARN] Detected jump back in time. Resetting RViz.
```

**Impact:** EKF nodes and RViz constantly resetting due to time synchronization issues
**Root Cause:** Simulation time not properly synchronized across nodes
**Frequency:** Continuous throughout execution

#### 2. **SDF Parsing Warnings** - 🟡 **MODERATE**
```
[gazebo-1] [Warning] XML Element[gz_frame_id], child of element[sensor], not defined in SDF
[gazebo-1] [Warning] XML Element[frame_id], child of element[lidar], not defined in SDF
```

**Impact:** Non-standard SDF elements being copied as children
**Root Cause:** URDF/SDF files using deprecated or non-standard Gazebo syntax
**Frequency:** Multiple occurrences during model loading

#### 3. **Qt Runtime Directory Warnings** - 🟡 **MODERATE**
```
[rviz2-8] QStandardPaths: runtime directory '/tmp/runtime-user' is not owned by UID 1000
[gazebo-1] QStandardPaths: runtime directory '/tmp/runtime-user' is not owned by UID 1000
```

**Impact:** Potential GUI rendering issues
**Root Cause:** Incorrect runtime directory ownership
**Frequency:** At GUI application startup

#### 4. **EGL Rendering Warnings** - 🟡 **MODERATE**
```
[gazebo-1] libEGL warning: egl: failed to create dri2 screen
```

**Impact:** GPU rendering fallback to software rendering
**Root Cause:** Missing or incompatible graphics drivers
**Frequency:** During Gazebo GUI initialization

---

## Drone SAR System Analysis

### ✅ **Successful Components**

| Component | Status | Description |
|-----------|--------|-------------|
| PX4 SITL | ✅ **SUCCESS** | PX4 Software-in-the-Loop started successfully |
| MAVROS | ✅ **SUCCESS** | All plugins loaded and initialized |
| MicroXRCE Agent | ✅ **SUCCESS** | DDS bridge running on port 8888 |
| MAVLink Connection | ✅ **SUCCESS** | UDP connection established |
| Static Transforms | ✅ **SUCCESS** | All coordinate frame transforms published |

### ⚠️ **Issues Identified**

#### 1. **Vehicle IMU Timestamp Errors** - 🔴 **CRITICAL**
```
[bash-1] ERROR [vehicle_imu] 0 - accel 1310988 timestamp error timestamp_sample: 106552000, previous timestamp_sample: 106552000
[bash-1] ERROR [vehicle_imu] 0 - gyro 1310988 timestamp error timestamp_sample: 106552000, previous timestamp_sample: 106552000
```

**Impact:** IMU sensor data synchronization issues affecting state estimation
**Root Cause:** PX4-Gazebo bridge timing synchronization problems
**Frequency:** Continuous throughout execution (200+ errors)

#### 2. **Missing Parameter Error** - 🟡 **MODERATE**
```
[bash-1] ERROR [param] Parameter UXRCE_DDS_CFG not found.
```

**Impact:** DDS configuration parameter missing
**Root Cause:** PX4 parameter not set in configuration
**Frequency:** Single occurrence during initialization

#### 3. **Deprecated Transform Arguments** - 🟡 **MODERATE**
```
[static_transform_publisher-*] [WARN] Old-style arguments are deprecated; see --help for new-style arguments
```

**Impact:** Using deprecated ROS2 static transform publisher syntax
**Root Cause:** Launch files using old positional argument format
**Frequency:** 7 occurrences during static transform publisher initialization

#### 4. **EKF Health Check Warning** - 🟡 **MODERATE**
```
[bash-1] WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data
```

**Impact:** EKF not receiving proper sensor data initially
**Root Cause:** Sensor initialization timing issues
**Frequency:** During system startup

---

## Solutions and Recommendations

### 🔧 **Immediate Fixes**

#### For Go2 SAR System:

1. **Fix Time Synchronization** (Priority: HIGH)
   ```xml
   <!-- Add to all launch files -->
   <param name="use_sim_time" value="true"/>
   ```

2. **Update URDF/SDF Files** (Priority: MEDIUM)
   ```bash
   # Replace deprecated elements in URDF files
   # Remove gz_frame_id tags or update to proper Gazebo Harmonic syntax
   ```

3. **Set Runtime Directory** (Priority: LOW)
   ```bash
   export XDG_RUNTIME_DIR=/tmp/runtime-$USER
   mkdir -p $XDG_RUNTIME_DIR
   ```

#### For Drone SAR System:

1. **Fix PX4 Parameters** (Priority: HIGH)
   ```bash
   # Add to PX4 startup script or parameter file
   param set UXRCE_DDS_CFG 0
   ```

2. **Update Launch Files** (Priority: MEDIUM)
   ```xml
   <!-- Replace old-style static transform publishers -->
   <!-- From: -->
   <node pkg="tf2_ros" exec="static_transform_publisher" args="0 0 0 0 0 0 1 map drone/odom"/>
   <!-- To: -->
   <node pkg="tf2_ros" exec="static_transform_publisher">
     <arg name="translation" value="0 0 0"/>
     <arg name="rotation" value="0 0 0 1"/>
     <arg name="parent_frame" value="map"/>
     <arg name="child_frame" value="drone/odom"/>
   </node>
   ```

3. **Improve Sensor Timing** (Priority: HIGH)
   ```bash
   # Add proper timing synchronization in PX4-Gazebo bridge
   # Ensure consistent timestamp handling
   ```

### 📊 **Performance Impact Assessment**

| System | Functionality | Performance Impact | User Experience |
|--------|---------------|-------------------|-----------------|
| Go2 SAR | ✅ Operational | 🟡 Minor lag due to TF resets | 🟡 Acceptable |
| Drone SAR | ✅ Operational | 🟡 IMU noise affects estimation | 🟡 Acceptable |

### 🔄 **Long-term Improvements**

1. **Implement proper simulation time handling across all nodes**
2. **Upgrade URDF/SDF files to Gazebo Harmonic standards**
3. **Add automated testing for launch file validation**
4. **Implement better error handling and recovery mechanisms**

---

## Conclusion

Both SAR systems launched successfully and are operational despite the identified warnings and errors. The issues are primarily related to:

- **Time synchronization** between simulation and ROS nodes
- **Deprecated syntax** in configuration files
- **Environmental setup** for GUI applications

**Recommendation:** Address the time synchronization issues first as they have the highest impact on system stability and performance. The other issues, while important for production deployment, do not prevent basic functionality.

---

## Appendix

### System Information
- **ROS2 Distribution:** Jazzy
- **Gazebo Version:** Harmonic
- **Test Date:** August 13, 2025
- **Test Duration:** 15 seconds
- **Launch Sequence:** Go2 → (6s delay) → Drone → (9s run) → Shutdown

### Files Analyzed
- Go2 SAR Launch: `ros2 launch sar_system go2_sar_system.launch.py`
- Drone SAR Launch: `ros2 launch sar_system drone_sar_system.launch.py`

### Next Steps
1. Apply recommended fixes
2. Re-test both systems
3. Monitor for improvement in error frequency
4. Document final configuration for production deployment