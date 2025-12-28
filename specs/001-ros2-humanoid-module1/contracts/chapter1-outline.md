# Chapter 1 Outline: ROS 2 as the Robotic Nervous System

**Priority**: P1 | **Word Target**: 1,200-1,400 | **Code Examples**: None

---

## Learning Objective

After reading this chapter, students can explain what a node, topic, and service are, and identify when to use reliable vs. best-effort QoS.

## Section Structure

### 1.1 Introduction: Why ROS 2? (150-200 words)
- Robot software complexity problem
- Middleware as solution
- ROS 2 as the "nervous system" analogy
- Brief history: ROS 1 → ROS 2

### 1.2 The ROS 2 Graph: Nodes and Communication (300-350 words)
- **Node definition**: Independent computational unit
- Node naming conventions
- Multiple nodes per robot
- Example: humanoid robot nodes (perception, planning, control)

**Concept Box**: Node
> A node is a single-purpose process that performs one job in the robot system. Each humanoid robot might have 10-50 nodes working together.

### 1.3 Topics: The Publish-Subscribe Pattern (300-350 words)
- **Topic definition**: Named message channel
- Publishers and subscribers
- Asynchronous communication
- One-to-many, many-to-one patterns

**Diagram**: ROS 2 Publish-Subscribe Architecture
```
┌─────────────┐    /joint_states    ┌─────────────┐
│   Sensor    │ ─────────────────▶  │  Planning   │
│    Node     │                     │    Node     │
└─────────────┘                     └─────────────┘
       │                                   │
       │        /imu_data                  │
       └─────────────────▶  ┌─────────────┐
                            │   Balance   │
                            │    Node     │
                            └─────────────┘
```

### 1.4 Services: Request-Response Communication (200-250 words)
- **Service definition**: Synchronous call pattern
- When to use services vs. topics
- Service naming conventions
- Example: humanoid calibration service

### 1.5 Quality of Service (QoS) (200-250 words)
- **QoS definition**: Delivery guarantees
- Reliability: best-effort vs. reliable
- Durability: volatile vs. transient-local
- Humanoid use case: sensor data (best-effort) vs. commands (reliable)

**Concept Box**: QoS Decision Guide
> Use **best-effort** for high-frequency sensor data where some loss is acceptable.
> Use **reliable** for commands and state changes that must be delivered.

### 1.6 Summary and Key Takeaways (100-150 words)
- Recap of node, topic, service, QoS
- Preview of Chapter 2 (Python implementation)
- Self-check questions

---

## Acceptance Criteria

- [ ] Word count: 1,200-1,400
- [ ] All 5 key concepts defined (Node, Topic, Service, QoS, DDS)
- [ ] 1 ASCII diagram included
- [ ] No code examples (conceptual chapter)
- [ ] FK readability: grade 11-13
- [ ] Self-check questions at end

## Verification Questions (for readers)

1. What is the difference between a topic and a service?
2. When would you use best-effort vs. reliable QoS?
3. How many nodes might a typical humanoid robot have?
4. What does DDS provide to ROS 2?
5. Can multiple nodes subscribe to the same topic?
