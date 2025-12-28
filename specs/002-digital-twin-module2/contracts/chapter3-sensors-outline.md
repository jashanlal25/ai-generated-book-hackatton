# Chapter 3 Outline: Sensor Simulation

**Priority**: P3 | **Word Target**: 900-1,300 | **Code Examples**: None (conceptual)

---

## Learning Objective

After reading this chapter, students can describe ray-casting for LiDAR, depth buffer approaches for cameras, and noise models for IMUs.

## Section Structure

### 3.1 Introduction: Why Simulate Sensors? (100-150 words)
- Synthetic data for ML training
- Testing perception algorithms before deployment
- Sensor-fusion development
- Domain gap challenges

### 3.2 LiDAR Simulation (250-350 words)
- **Ray-casting**: Tracing beams from sensor origin
- Beam divergence and footprint size
- Return intensity modeling
- Multi-echo and ghost returns
- Noise models: range noise, angular noise

**Concept Box**: Ray-Casting
> Ray-casting traces virtual laser beams from the sensor, calculating intersection points with scene geometry to determine range measurements.

**Diagram**: Sensor Data Flow
```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   Physical   │───▶│   Sensor     │───▶│   Noise      │
│   World      │    │   Model      │    │   Model      │
└──────────────┘    └──────────────┘    └──────────────┘
                                               │
                                               ▼
                          ┌──────────────────────────────┐
                          │   Simulated Sensor Output    │
                          │   (Point Cloud / Image / IMU)│
                          └──────────────────────────────┘
```

### 3.3 Depth Camera Simulation (200-300 words)
- **Structured light** (Intel RealSense): Projector + camera
- **Time-of-Flight (ToF)**: Phase-shift measurement
- Depth buffer approach in simulators
- Artifacts: flying pixels, multi-path interference
- Noise models for each technology

**Concept Box**: Depth Buffer
> The depth buffer stores the distance from the camera to each pixel, enabling efficient depth image generation without per-ray calculations.

### 3.4 IMU Simulation (200-300 words)
- Accelerometer and gyroscope fundamentals
- **Bias**: Systematic offset in measurements
- **Drift**: Accumulating error over time (random walk)
- Allan variance characterization
- Calibration in simulation vs. real sensors

### 3.5 Bridging the Sim-to-Real Gap (100-150 words)
- Domain randomization
- Realistic noise injection
- Sensor calibration matching
- Validation against real sensor data

### 3.6 Summary and Key Takeaways (50-100 words)
- Recap of sensor simulation approaches
- Preview of environment building (Chapter 4)
- Self-check questions

---

## Acceptance Criteria

- [ ] Word count: 900-1,300
- [ ] All 6 key concepts defined (Ray-Casting, Depth Buffer, Noise Model, Beam Divergence, Gyroscope Drift, Accelerometer Bias)
- [ ] 1 ASCII diagram included
- [ ] At least 3 citations (minimum 2 peer-reviewed)
- [ ] No code examples (conceptual chapter)
- [ ] FK readability: grade 11-13

## Required Citations

1. Manivasagam, S., et al. (2020). LiDARsim: Realistic LiDAR simulation. *CVPR*.
2. Nguyen, C. V., et al. (2012). Modeling kinect sensor noise. *3DIM/3DPVT*.
3. Woodman, O. J. (2007). An introduction to inertial navigation. *Cambridge Technical Report*.

## Verification Questions (for readers)

1. What is the difference between ray-casting and depth buffer approaches?
2. Why do ToF cameras experience multi-path interference?
3. What causes gyroscope drift over time?
4. How can domain randomization help bridge the sim-to-real gap?
