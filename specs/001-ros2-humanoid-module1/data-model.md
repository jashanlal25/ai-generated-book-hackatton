# Data Model: ROS 2 Fundamentals for Humanoid Robotics (Module 1)

**Branch**: `001-ros2-humanoid-module1` | **Date**: 2025-12-09

---

## Content Entities

### Chapter Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string | `chapter-{n}` format | Unique identifier |
| title | string | 5-12 words | Chapter heading |
| slug | string | kebab-case | URL-safe identifier |
| word_count | integer | 1,000-1,800 | Total word count |
| priority | enum | P1, P2, P3 | Implementation priority |
| learning_objective | string | 1 sentence | What reader will learn |
| prerequisites | string[] | Chapter IDs | Required prior chapters |
| concepts | Concept[] | Min 3 | Key terms introduced |
| diagrams | Diagram[] | Min 1 | Visual elements |
| code_examples | CodeExample[] | Varies | Runnable code |
| summary | string | 2-3 sentences | Chapter takeaways |

### Concept Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| term | string | Unique per chapter | Technical term |
| definition | string | 1-2 sentences | Plain language definition |
| ros2_reference | string | URL | Official documentation link |
| first_mention_line | integer | > 0 | Line where term is introduced |

### Diagram Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string | `diagram-{chapter}-{n}` | Unique identifier |
| type | enum | ascii, markdown-table | Rendering format |
| title | string | Required | Caption/description |
| content | string | Valid ASCII art | Diagram content |
| alt_text | string | Required | Accessibility description |

### CodeExample Entity

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string | `code-{chapter}-{n}` | Unique identifier |
| filename | string | Valid filename | Source file name |
| language | enum | python, xml, bash | Syntax highlighting |
| ros2_version | string | "humble" | Target ROS 2 distribution |
| complete | boolean | true | No partial snippets |
| tested | boolean | true after validation | Execution verified |
| dependencies | string[] | Package names | Required ROS 2 packages |
| lines_of_code | integer | Tracked | Complexity indicator |

---

## Chapter Instances

### Chapter 1: ROS 2 as the Robotic Nervous System

```yaml
id: chapter-1
title: "ROS 2 as the Robotic Nervous System"
slug: ros2-overview
word_count: 1200-1400
priority: P1
learning_objective: "Understand ROS 2 middleware architecture and core communication patterns"
prerequisites: []
concepts:
  - term: "Node"
    definition: "An independent process that performs computation in ROS 2"
  - term: "Topic"
    definition: "A named bus for asynchronous message passing between nodes"
  - term: "Service"
    definition: "A synchronous request-response communication pattern"
  - term: "QoS"
    definition: "Quality of Service policies controlling message delivery guarantees"
  - term: "DDS"
    definition: "Data Distribution Service, the middleware standard underlying ROS 2"
diagrams:
  - id: diagram-1-1
    type: ascii
    title: "ROS 2 Publish-Subscribe Architecture"
code_examples: []  # Conceptual chapter
```

### Chapter 2: Building ROS 2 Nodes with Python

```yaml
id: chapter-2
title: "Building ROS 2 Nodes with Python"
slug: rclpy-nodes
word_count: 1400-1800
priority: P2
learning_objective: "Create functional publisher, subscriber, and service nodes using rclpy"
prerequisites: [chapter-1]
concepts:
  - term: "rclpy"
    definition: "The Python client library for ROS 2"
  - term: "Publisher"
    definition: "A node component that sends messages to a topic"
  - term: "Subscriber"
    definition: "A node component that receives messages from a topic"
  - term: "Callback"
    definition: "A function executed when a message or request arrives"
diagrams:
  - id: diagram-2-1
    type: ascii
    title: "Humanoid Joint State Message Flow"
code_examples:
  - id: code-2-1
    filename: publisher.py
    language: python
    dependencies: [rclpy, std_msgs]
  - id: code-2-2
    filename: subscriber.py
    language: python
    dependencies: [rclpy, std_msgs]
  - id: code-2-3
    filename: service_server.py
    language: python
    dependencies: [rclpy, example_interfaces]
  - id: code-2-4
    filename: service_client.py
    language: python
    dependencies: [rclpy, example_interfaces]
```

### Chapter 3: Describing Humanoid Robots with URDF

```yaml
id: chapter-3
title: "Describing Humanoid Robots with URDF"
slug: urdf-basics
word_count: 1200-1600
priority: P3
learning_objective: "Define robot physical structure using URDF elements"
prerequisites: [chapter-1]
concepts:
  - term: "URDF"
    definition: "Unified Robot Description Format, an XML specification for robot models"
  - term: "Link"
    definition: "A rigid body segment in the robot kinematic chain"
  - term: "Joint"
    definition: "A connection between links defining allowed motion"
  - term: "Visual"
    definition: "The rendered appearance of a link"
  - term: "Collision"
    definition: "The simplified geometry used for physics calculations"
diagrams:
  - id: diagram-3-1
    type: ascii
    title: "Humanoid Torso URDF Tree"
code_examples:
  - id: code-3-1
    filename: humanoid_torso.urdf
    language: xml
    dependencies: []
```

---

## Validation Rules

### Word Count

```
IF chapter.word_count < 1000 THEN ERROR "Below minimum"
IF chapter.word_count > 1800 THEN ERROR "Exceeds maximum"
```

### Code Completeness

```
FOR EACH code_example IN chapter.code_examples:
  IF code_example.complete != true THEN ERROR "Partial code not allowed"
  IF code_example.tested != true THEN WARNING "Code not yet validated"
```

### Concept Coverage

```
FOR EACH concept IN chapter.concepts:
  IF concept.first_mention_line > (chapter.word_count / 10) THEN
    WARNING "Concept defined late in chapter"
```

### Diagram Accessibility

```
FOR EACH diagram IN chapter.diagrams:
  IF diagram.alt_text IS EMPTY THEN ERROR "Missing accessibility text"
```

---

## State Transitions

```
Chapter States:
  draft → review → validated → published

Transitions:
  draft → review: All content complete, diagrams present
  review → validated: Code tested, word count verified, FK checked
  validated → published: Technical review approved
```
