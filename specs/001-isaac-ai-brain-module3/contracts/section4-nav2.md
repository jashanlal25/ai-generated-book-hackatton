# Contract: Section 4 - Nav2 Path Planning

## Metadata
- **Word Target**: 1,000 words
- **Priority**: P3
- **Dependencies**: Section 3

## Learning Objectives
1. Explain Nav2 architecture and components
2. Describe costmap concepts
3. Understand bipedal navigation challenges

## Required Content

### 4.1 Nav2 Architecture
- Planner server (global path)
- Controller server (local trajectory)
- Recovery server (stuck behaviors)
- Behavior trees for orchestration

### 4.2 Costmaps
- Global vs local costmaps
- Obstacle representation
- Inflation for safety margins

### 4.3 Bipedal Navigation Challenges
- Wheeled vs bipedal comparison
- Footstep planning concepts
- Balance and stability (ZMP)
- Terrain assessment

## Required Diagram
Nav2 architecture showing component interactions.

## Required Citations
- Macenski, S., et al. (2020). Marathon 2. *IROS*.
- Kajita, S., et al. (2014). Humanoid robotics. *Springer*.
- Open Robotics. (2024). Nav2 documentation.

## Acceptance Criteria
- [ ] Explains Nav2 components clearly
- [ ] Defines costmaps with examples
- [ ] Contrasts bipedal vs wheeled navigation
- [ ] Peer-reviewed citations present
