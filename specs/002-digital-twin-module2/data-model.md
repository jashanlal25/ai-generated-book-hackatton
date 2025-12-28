# Data Model: The Digital Twin - Gazebo & Unity Simulation (Module 2)

**Branch**: `002-digital-twin-module2` | **Date**: 2025-12-09

---

## Content Entities

### Chapter Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string | `chapter-{n}` format | Unique identifier |
| title | string | 5-12 words | Chapter heading |
| slug | string | kebab-case | URL-safe identifier |
| word_count | integer | Varies by chapter | Total word count |
| priority | enum | P1, P2, P3, P4 | Implementation priority |
| learning_objective | string | 1 sentence | What reader will learn |
| prerequisites | string[] | Chapter IDs | Required prior chapters |
| concepts | Concept[] | Min 3 | Key terms introduced |
| diagrams | Diagram[] | Min 1 | Visual elements |
| citations | Citation[] | Min 2 | Academic/official sources |
| summary | string | 2-3 sentences | Chapter takeaways |

### Concept Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| term | string | Unique per chapter | Technical term |
| definition | string | 1-2 sentences | Plain language definition |
| source | string | Citation key | Source for definition |
| first_mention_line | integer | > 0 | Line where term is introduced |

### Citation Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| key | string | author-year format | Citation identifier |
| type | enum | peer-reviewed, official-doc | Source classification |
| apa_format | string | APA 7th edition | Full citation text |
| url | string | Optional | DOI or URL |

---

## Chapter Instances

### Chapter 1: Gazebo Physics Simulation

```yaml
id: chapter-1
title: "Gazebo Physics Simulation"
slug: gazebo-physics
word_count: 800-1200
priority: P1
learning_objective: "Understand how physics engines simulate gravity, collision, and rigid body dynamics"
prerequisites: [module-1-chapter-3]  # URDF from Module 1
concepts:
  - term: "Physics Engine"
    definition: "Software that calculates forces, collisions, and motion in a simulated environment"
  - term: "Rigid Body Dynamics"
    definition: "Mathematical modeling of solid objects that don't deform under force"
  - term: "Collision Detection"
    definition: "Algorithms that determine when objects intersect in 3D space"
  - term: "Solver Iterations"
    definition: "Number of computational passes to resolve physics constraints"
  - term: "Time Step"
    definition: "Duration of each simulation update cycle"
diagrams:
  - id: diagram-1-1
    type: ascii
    title: "Physics Simulation Pipeline"
citations:
  - key: koenig-2004
    type: peer-reviewed
  - key: gazebo-docs-2024
    type: official-doc
```

### Chapter 2: Unity Rendering and Human-Robot Interaction

```yaml
id: chapter-2
title: "Unity Rendering and Human-Robot Interaction"
slug: unity-rendering
word_count: 700-1000
priority: P2
learning_objective: "Understand Unity rendering pipelines and human-robot interaction simulation"
prerequisites: [chapter-1]
concepts:
  - term: "Rendering Pipeline"
    definition: "Sequence of stages that transform 3D scene data into 2D images"
  - term: "URP (Universal Render Pipeline)"
    definition: "Unity's optimized pipeline for cross-platform performance"
  - term: "HDRP (High Definition Render Pipeline)"
    definition: "Unity's pipeline for photorealistic rendering on high-end hardware"
  - term: "Human-Robot Interaction (HRI)"
    definition: "Study and design of how humans and robots communicate and collaborate"
diagrams:
  - id: diagram-2-1
    type: ascii
    title: "Unity Rendering Pipeline"
citations:
  - key: unity-urp-docs-2024
    type: official-doc
  - key: unity-robotics-hub-2024
    type: official-doc
```

### Chapter 3: Sensor Simulation

```yaml
id: chapter-3
title: "Sensor Simulation"
slug: sensor-simulation
word_count: 900-1300
priority: P3
learning_objective: "Understand how simulators model LiDAR, depth cameras, and IMUs"
prerequisites: [chapter-1]
concepts:
  - term: "Ray-Casting"
    definition: "Technique that traces rays from a source to detect object intersections"
  - term: "Depth Buffer"
    definition: "GPU memory storing distance from camera to each pixel"
  - term: "Sensor Noise Model"
    definition: "Mathematical representation of measurement errors and uncertainties"
  - term: "Beam Divergence"
    definition: "Spreading of a laser beam over distance"
  - term: "Gyroscope Drift"
    definition: "Gradual accumulation of angular measurement errors over time"
  - term: "Accelerometer Bias"
    definition: "Systematic offset in acceleration measurements"
diagrams:
  - id: diagram-3-1
    type: ascii
    title: "Sensor Data Flow"
citations:
  - key: manivasagam-2020
    type: peer-reviewed
  - key: nguyen-2012
    type: peer-reviewed
  - key: woodman-2007
    type: peer-reviewed
```

### Chapter 4: Environment Building

```yaml
id: chapter-4
title: "Environment Building"
slug: environment-building
word_count: 600-900
priority: P4
learning_objective: "Understand simulation world construction principles"
prerequisites: [chapter-1, chapter-2]
concepts:
  - term: "SDF (Simulation Description Format)"
    definition: "XML format for describing robots, objects, and environments in Gazebo"
  - term: "Heightmap"
    definition: "Grayscale image used to generate 3D terrain geometry"
  - term: "Physics Mesh"
    definition: "Simplified geometry used for collision detection"
  - term: "Asset Management"
    definition: "Organization and reuse of 3D models, textures, and materials"
diagrams:
  - id: diagram-4-1
    type: ascii
    title: "World File Structure"
citations:
  - key: sdformat-spec-2024
    type: official-doc
  - key: gazebo-docs-2024
    type: official-doc
```

---

## Validation Rules

### Word Count

```
chapter-1: 800 <= word_count <= 1200
chapter-2: 700 <= word_count <= 1000
chapter-3: 900 <= word_count <= 1300
chapter-4: 600 <= word_count <= 900
total: 3000 <= sum(chapters) <= 5000
```

### Citation Balance

```
peer_reviewed_count = count(citations WHERE type == "peer-reviewed")
total_count = count(citations)
IF peer_reviewed_count / total_count < 0.5 THEN ERROR "Insufficient peer-reviewed sources"
```

### Concept Coverage

```
FOR EACH concept IN chapter.concepts:
  IF concept.source IS EMPTY THEN ERROR "Concept needs citation"
  IF concept.first_mention_line > (chapter.word_count / 5) THEN
    WARNING "Concept defined late in chapter"
```

---

## State Transitions

```
Chapter States:
  draft → review → validated → published

Transitions:
  draft → review: All content complete, citations present
  review → validated: Sources verified, word count checked, plagiarism 0%
  validated → published: Technical review approved
```
