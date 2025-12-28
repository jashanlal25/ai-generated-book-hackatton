# Chapter 3 Outline: Describing Humanoid Robots with URDF

**Priority**: P3 | **Word Target**: 1,200-1,600 | **Code Examples**: 1 (URDF)

---

## Learning Objective

After reading this chapter, students can define robot physical structure using URDF elements and create a valid URDF file for a simple humanoid torso with links and joints.

## Section Structure

### 3.1 Introduction to Robot Description (150-200 words)
- Why formal robot descriptions matter
- URDF: Unified Robot Description Format
- XML-based specification
- Use cases: visualization, simulation, motion planning

### 3.2 Links: The Building Blocks (250-300 words)
- **Link definition**: A rigid body segment
- Required elements: `<link name="...">`
- Visual geometry: what the robot looks like
- Collision geometry: simplified physics shape
- Inertial properties: mass and moments of inertia

**Concept Box**: Visual vs. Collision
> Visual geometry can be detailed meshes for appearance. Collision geometry should be simple primitives (box, cylinder, sphere) for efficient physics computation.

### 3.3 Joints: Connecting the Parts (300-350 words)
- **Joint definition**: Connection between two links
- Parent and child links
- Joint types: fixed, revolute, prismatic, continuous
- Joint limits: position, velocity, effort
- Origin and axis specification

**Table**: Joint Types for Humanoid Robots

| Joint Type | Motion | Humanoid Use Case |
|------------|--------|-------------------|
| `revolute` | Rotation with limits | Shoulder, elbow, knee |
| `fixed` | No motion | Sensor mounting |
| `prismatic` | Linear translation | Linear actuators |
| `continuous` | Unlimited rotation | Not typical |

### 3.4 Building a Humanoid Torso (350-450 words)
- Step-by-step construction
- Base link (torso)
- Shoulder joint and upper arm
- Elbow joint and forearm
- Realistic joint limits

**Diagram**: Humanoid Torso URDF Tree
```
                    torso_link
                        │
          ┌─────────────┼─────────────┐
          │             │             │
    left_shoulder  head_mount   right_shoulder
    (revolute)      (fixed)      (revolute)
          │                           │
    left_upper_arm             right_upper_arm
          │                           │
    left_elbow                  right_elbow
    (revolute)                  (revolute)
          │                           │
    left_forearm               right_forearm
```

**Code Example 1**: humanoid_torso.urdf
```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">

  <!-- Base Link: Torso -->
  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Right Shoulder Joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm_link"/>
    <origin xyz="0.25 0 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>

  <!-- Right Upper Arm -->
  <link name="right_upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Elbow Joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm_link"/>
    <child link="right_forearm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="30" velocity="2.0"/>
  </joint>

  <!-- Right Forearm -->
  <link name="right_forearm_link">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.003"/>
    </inertial>
  </link>

</robot>
```

### 3.5 Validating Your URDF (150-200 words)
- `check_urdf` command
- Common errors and fixes
- XML syntax validation
- Visualization with `urdf_to_graphviz`

**Commands**:
```bash
# Validate URDF structure
check_urdf humanoid_torso.urdf

# Generate visual graph
urdf_to_graphviz humanoid_torso.urdf
```

### 3.6 Summary and Extensions (100-150 words)
- Recap of links, joints, URDF structure
- Next steps: xacro for parameterization
- Preview of simulation (future modules)

---

## Acceptance Criteria

- [ ] Word count: 1,200-1,600
- [ ] 1 complete, valid URDF file
- [ ] URDF passes `check_urdf` validation
- [ ] 1 ASCII diagram (URDF tree)
- [ ] Joint types table included
- [ ] FK readability: grade 11-13

## Validation Commands

```bash
# Install URDF tools (if not present)
sudo apt install liburdfdom-tools

# Validate URDF
check_urdf humanoid_torso.urdf

# Expected output:
# robot name is: humanoid_torso
# ---------- Successfully Parsed XML ---------------
# root Link: torso_link has 1 child(ren)
#     child(1):  right_upper_arm_link
#         child(1):  right_forearm_link
```

## URDF Elements Reference

| Element | Purpose | Required |
|---------|---------|----------|
| `<robot>` | Root container | Yes |
| `<link>` | Body segment | Yes (min 1) |
| `<joint>` | Link connection | If >1 link |
| `<visual>` | Appearance | No |
| `<collision>` | Physics shape | No |
| `<inertial>` | Mass properties | No |
| `<origin>` | Position/orientation | No |
| `<axis>` | Motion axis | For movable joints |
| `<limit>` | Joint constraints | For revolute/prismatic |
