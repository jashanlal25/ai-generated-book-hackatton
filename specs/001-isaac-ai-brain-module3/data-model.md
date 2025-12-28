# Data Model: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Date**: 2025-12-09 | **Plan**: [plan.md](./plan.md)

## Content Entities

### Primary Concepts

| Entity | Type | Definition | Relationships |
|--------|------|------------|---------------|
| Isaac Sim | Platform | NVIDIA's Omniverse-based simulator | Contains: Replicator, ROS 2 Bridge |
| Replicator | Framework | Synthetic data generation tool | Part of: Isaac Sim |
| Domain Randomization | Technique | Variation of simulation parameters | Used by: Replicator |
| cuVSLAM | Algorithm | GPU-accelerated Visual SLAM | Part of: Isaac ROS |
| Isaac ROS | Package Suite | GPU-accelerated ROS 2 perception | Integrates with: Nav2 |
| Nav2 | Stack | ROS 2 navigation framework | Uses: Costmaps, Planners |
| Costmap | Data Structure | 2D obstacle grid representation | Used by: Nav2 Planners |
| Behavior Tree | Pattern | Task orchestration structure | Controls: Nav2 navigation |

### Section-Entity Mapping

| Section | Primary Entities | Word Target |
|---------|-----------------|-------------|
| 1. Isaac Sim Overview | Isaac Sim, Omniverse, PhysX | 1,000 |
| 2. Synthetic Data | Replicator, Domain Randomization | 800 |
| 3. Isaac ROS VSLAM | cuVSLAM, Visual Odometry | 1,000 |
| 4. Nav2 Planning | Nav2, Costmap, Behavior Tree | 1,000 |
| 5. Integration | All entities combined | 500 |

## Diagram Specifications

### Diagram 1: Sim-to-Real Pipeline
```
[Isaac Sim] --> [Synthetic Data] --> [Model Training] --> [Isaac ROS] --> [Real Robot]
     |               |                     |                   |
     v               v                     v                   v
  Physics      Ground Truth           Trained DNN       GPU Inference
  Rendering    Labels                                   cuVSLAM/Nav2
```

### Diagram 2: VSLAM Data Flow
```
[Camera] --> [Feature Extraction] --> [Visual Odometry] --> [Pose Estimate]
                    |                        |                    |
                    v                        v                    v
              [Landmarks]            [Loop Closure]          [Map Update]
```

### Diagram 3: Nav2 Architecture
```
[Goal] --> [BT Navigator] --> [Planner Server] --> [Global Path]
                |                                        |
                v                                        v
        [Controller Server] <-- [Costmap 2D] <-- [Local Path]
                |
                v
        [cmd_vel to Robot]
```

## Citation Requirements

| Claim Type | Required Source |
|------------|-----------------|
| Isaac Sim features | NVIDIA official docs |
| VSLAM algorithms | Peer-reviewed paper |
| Nav2 architecture | Macenski et al. (2020) |
| Bipedal challenges | Kajita et al. (2014) |
| Domain randomization | Tobin et al. (2017) |
