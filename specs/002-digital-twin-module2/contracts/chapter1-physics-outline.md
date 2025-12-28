# Chapter 1 Outline: Gazebo Physics Simulation

**Priority**: P1 | **Word Target**: 800-1,200 | **Code Examples**: None (conceptual)

---

## Learning Objective

After reading this chapter, students can explain how physics engines simulate gravity, collision detection, and rigid body dynamics in Gazebo.

## Section Structure

### 1.1 Introduction: Why Physics Simulation Matters (100-150 words)
- Digital twins require accurate physics
- Testing before deployment on real robots
- Cost and safety benefits of simulation

### 1.2 Physics Engines in Gazebo (200-250 words)
- **ODE (Open Dynamics Engine)**: Default engine, general-purpose
- **Bullet**: GPU acceleration, soft body support
- **DART**: Research-focused, constraint solving
- When to choose each engine

**Concept Box**: Physics Engine
> A physics engine calculates forces, collisions, and motion for every object in the simulated world, updating positions and velocities at each time step.

### 1.3 Rigid Body Dynamics (200-250 words)
- Newton-Euler equations
- Mass, inertia tensors, center of mass
- Forces and torques
- Integration methods (explicit, implicit)

**Diagram**: Physics Simulation Pipeline
```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Forces    │────▶│  Dynamics   │────▶│  Position   │
│   Applied   │     │  Solver     │     │  Update     │
└─────────────┘     └─────────────┘     └─────────────┘
       │                   │                   │
       │                   ▼                   │
       │           ┌─────────────┐             │
       └──────────▶│  Collision  │◀────────────┘
                   │  Detection  │
                   └─────────────┘
```

### 1.4 Collision Detection and Response (200-300 words)
- **Broad phase**: Axis-aligned bounding boxes (AABB), spatial hashing
- **Narrow phase**: GJK, EPA algorithms
- Contact manifolds and collision response
- Mesh simplification for performance

### 1.5 Simulation Parameters (100-150 words)
- **Time step**: Trade-off between accuracy and performance
- **Solver iterations**: Constraint satisfaction accuracy
- **ERP/CFM**: Error reduction and constraint force mixing
- Tuning for humanoid robots

### 1.6 Summary and Key Takeaways (50-100 words)
- Recap of physics engine concepts
- Preview of Unity rendering (Chapter 2)
- Self-check questions

---

## Acceptance Criteria

- [ ] Word count: 800-1,200
- [ ] All 5 key concepts defined (Physics Engine, Rigid Body, Collision Detection, Time Step, Solver)
- [ ] 1 ASCII diagram included
- [ ] At least 2 citations (1 peer-reviewed minimum)
- [ ] No code examples (conceptual chapter)
- [ ] FK readability: grade 11-13

## Required Citations

1. Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo. *IEEE/RSJ IROS*.
2. Open Robotics. (2024). *Gazebo Sim documentation*.

## Verification Questions (for readers)

1. What is the difference between broad-phase and narrow-phase collision detection?
2. Why does time step size affect simulation accuracy?
3. Which physics engine would you choose for GPU-accelerated simulation?
4. What happens if solver iterations are too low?
