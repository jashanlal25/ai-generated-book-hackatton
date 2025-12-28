# Quickstart: Module 2 Implementation Guide

**Branch**: `002-digital-twin-module2` | **Date**: 2025-12-09

---

## Prerequisites

### Knowledge Requirements

- Completed Module 1 (ROS 2 Fundamentals) or equivalent
- Basic physics understanding (Newton's laws, forces, vectors)
- Familiarity with 3D coordinate systems

### Tools

- Docusaurus 3.x installed (for building the book)
- Text editor with Markdown support
- Word count tool (e.g., `wc -w` on Linux/Mac)

---

## Implementation Workflow

### Step 1: Create Module Directory Structure

```bash
# From repository root
mkdir -p my-website/docs/module-2-digital-twin
```

### Step 2: Create Category Configuration

Create `my-website/docs/module-2-digital-twin/_category_.json`:

```json
{
  "label": "Module 2: The Digital Twin",
  "position": 2,
  "link": {
    "type": "generated-index",
    "description": "Learn physics simulation, rendering, and sensor modeling for humanoid robotics."
  }
}
```

### Step 3: Implement Chapters (Priority Order)

| Priority | Chapter | File | Word Target |
|----------|---------|------|-------------|
| P1 | Gazebo Physics | chapter-1-gazebo-physics.mdx | 800-1,200 |
| P2 | Unity Rendering | chapter-2-unity-rendering.mdx | 700-1,000 |
| P3 | Sensor Simulation | chapter-3-sensor-simulation.mdx | 900-1,300 |
| P4 | Environment Building | chapter-4-environment-building.mdx | 600-900 |

### Step 4: Writing Guidelines

**For Each Chapter:**

1. Start with learning objective clearly stated
2. Define concepts in plain language before technical details
3. Use ASCII diagrams (from contracts/ outlines)
4. Include all required citations in APA format
5. End with summary and self-check questions

---

## Content Guidelines

### Writing Style

- **Active voice**: "The physics engine calculates" not "Forces are calculated by"
- **Second person**: "You will learn" not "The reader will learn"
- **Present tense**: "This chapter covers" not "This chapter will cover"
- **Technical but accessible**: Assume basic physics knowledge only

### Citation Format (APA 7th Edition)

**In-text citation**:
```
Physics engines like ODE use iterative solvers (Koenig & Howard, 2004).
```

**Reference list entry**:
```
Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo,
    an open-source multi-robot simulator. In *IEEE/RSJ International
    Conference on Intelligent Robots and Systems* (pp. 2149-2154).
```

### ASCII Diagrams

- Use box-drawing characters: `┌ ─ ┐ │ └ ┘ ├ ┤ ┬ ┴ ┼`
- Maximum width: 70 characters
- Include alt-text description above diagram

### Concept Definitions

```mdx
:::info Definition
**Physics Engine**: Software that calculates forces, collisions, and motion
in a simulated environment at each time step.
:::
```

---

## Source Requirements

### Citation Balance

| Type | Requirement | Examples |
|------|-------------|----------|
| Peer-reviewed | ≥50% of total | IEEE papers, ICRA/IROS proceedings |
| Official docs | ≤50% of total | Gazebo Sim, Unity documentation |
| Total per chapter | ≥2 citations | - |
| Module total | ≥10 citations | - |

### Required Sources by Chapter

**Chapter 1 (Physics)**:
- Koenig & Howard (2004) - Gazebo design
- Gazebo Sim documentation

**Chapter 2 (Unity)**:
- Unity URP/HDRP documentation
- Unity Robotics Hub

**Chapter 3 (Sensors)**:
- Manivasagam et al. (2020) - LiDAR simulation
- Nguyen et al. (2012) - Depth camera noise
- Woodman (2007) - IMU navigation

**Chapter 4 (Environments)**:
- SDFormat specification
- Gazebo Sim documentation

---

## Validation Procedures

### Word Count Verification

```bash
# For MDX files
wc -w my-website/docs/module-2-digital-twin/chapter-*.mdx

# Per-chapter targets:
# Chapter 1: 800-1,200 words
# Chapter 2: 700-1,000 words
# Chapter 3: 900-1,300 words
# Chapter 4: 600-900 words
# Total: 3,000-5,000 words
```

### Docusaurus Build Test

```bash
cd my-website
npm run build

# Should complete without errors
```

### Citation Audit

For each citation:
1. Verify paper/doc exists at cited URL/DOI
2. Confirm APA format is correct
3. Check that ≥50% are peer-reviewed

---

## Common Issues and Solutions

| Issue | Solution |
|-------|----------|
| Diagram misaligned | Use monospace preview; check character width |
| Citation format wrong | Use APA 7th edition templates |
| Word count too high | Cut redundant explanations; tighten prose |
| Word count too low | Add examples; expand concept explanations |
| Docusaurus build fails | Check MDX syntax; verify closing tags |

---

## File Checklist

### Required Deliverables

```
my-website/
├── docs/
│   └── module-2-digital-twin/
│       ├── _category_.json              [ ]
│       ├── chapter-1-gazebo-physics.mdx     [ ] 800-1,200 words
│       ├── chapter-2-unity-rendering.mdx    [ ] 700-1,000 words
│       ├── chapter-3-sensor-simulation.mdx  [ ] 900-1,300 words
│       └── chapter-4-environment-building.mdx [ ] 600-900 words
```

### Verification Sign-off

| Check | Passed | Date | Verifier |
|-------|--------|------|----------|
| Word counts in range | [ ] | | |
| All citations present | [ ] | | |
| ≥50% peer-reviewed | [ ] | | |
| Diagrams render | [ ] | | |
| Docusaurus builds | [ ] | | |
| FK readability 11-13 | [ ] | | |
| Technical review | [ ] | | |
