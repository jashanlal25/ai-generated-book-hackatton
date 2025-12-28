# Quickstart: Module 1 Implementation Guide

**Branch**: `001-ros2-humanoid-module1` | **Date**: 2025-12-09

---

## Prerequisites

### Environment Setup

- **Operating System**: Ubuntu 22.04 LTS (or WSL2 on Windows)
- **ROS 2 Distribution**: Humble Hawksbill
- **Python**: 3.10+
- **Node.js**: 18+ (for Docusaurus)

### Verify ROS 2 Installation

```bash
# Check ROS 2 is sourced
echo $ROS_DISTRO
# Expected: humble

# Verify rclpy is available
python3 -c "import rclpy; print('rclpy available')"

# Check URDF tools
which check_urdf
# If missing: sudo apt install liburdfdom-tools
```

### Verify Docusaurus Setup

```bash
cd my-website
npm install
npm run build  # Should complete without errors
```

---

## Implementation Workflow

### Step 1: Create Module Directory Structure

```bash
# From repository root
mkdir -p my-website/docs/module-1-ros2-fundamentals
mkdir -p my-website/static/code-examples/module-1
```

### Step 2: Create Category Configuration

Create `my-website/docs/module-1-ros2-fundamentals/_category_.json`:

```json
{
  "label": "Module 1: ROS 2 Fundamentals",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Learn ROS 2 middleware fundamentals for humanoid robotics."
  }
}
```

### Step 3: Implement Chapters (Priority Order)

| Priority | Chapter | File | Word Target |
|----------|---------|------|-------------|
| P1 | ROS 2 Overview | chapter-1-ros2-overview.mdx | 1,200-1,400 |
| P2 | rclpy Nodes | chapter-2-rclpy-nodes.mdx | 1,400-1,800 |
| P3 | URDF Basics | chapter-3-urdf-basics.mdx | 1,200-1,600 |

### Step 4: Code Example Workflow

For each code example in contracts/:

1. **Write code** to `my-website/static/code-examples/module-1/`
2. **Test locally** on ROS 2 Humble
3. **Embed in chapter** using code fence with title

```mdx
```python title="static/code-examples/module-1/publisher.py"
# Code here
```
```

### Step 5: Validation Checklist

For each chapter:

- [ ] Word count within range (use `wc -w`)
- [ ] Code examples complete (no `...` or `# more code`)
- [ ] Code tested on ROS 2 Humble
- [ ] Diagram renders correctly in Docusaurus
- [ ] FK readability grade 11-13
- [ ] APA citations for external sources

---

## Content Guidelines

### Writing Style

- **Active voice**: "The node publishes" not "Messages are published by the node"
- **Second person**: "You will create" not "The reader will create"
- **Present tense**: "This chapter covers" not "This chapter will cover"
- **Concise**: Avoid filler phrases ("It is important to note that...")

### Code Block Format

```mdx
```python title="filename.py" showLineNumbers
#!/usr/bin/env python3
import rclpy
# Complete, runnable code only
```
```

### ASCII Diagrams

- Use box-drawing characters: `┌ ─ ┐ │ └ ┘ ├ ┤ ┬ ┴ ┼`
- Maximum width: 70 characters
- Include alt-text description above diagram

### Concept Definitions

```mdx
:::info Definition
**Node**: An independent process that performs computation in ROS 2.
:::
```

---

## Testing Procedures

### Code Execution Test

```bash
# Set up ROS 2 environment
source /opt/ros/humble/setup.bash

# Create temporary workspace
mkdir -p ~/ros2_book_test/src
cd ~/ros2_book_test

# Copy code examples
cp my-website/static/code-examples/module-1/*.py src/

# Run publisher (Terminal 1)
python3 src/publisher.py

# Run subscriber (Terminal 2)
python3 src/subscriber.py

# Expected: Messages flow between terminals
```

### URDF Validation Test

```bash
# Validate URDF structure
check_urdf my-website/static/code-examples/module-1/humanoid_torso.urdf

# Expected output: "Successfully Parsed XML"
```

### Word Count Verification

```bash
# For MDX files (excluding code blocks)
# Simple approach:
wc -w my-website/docs/module-1-ros2-fundamentals/chapter-*.mdx

# Each file should be 1,000-1,800 words
```

### Docusaurus Build Test

```bash
cd my-website
npm run build

# Should complete without errors
# Check generated HTML in build/ directory
```

---

## Common Issues and Solutions

| Issue | Solution |
|-------|----------|
| `rclpy` import error | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| URDF validation fails | Check XML syntax, ensure all joints have parent/child |
| Docusaurus build fails | Verify MDX syntax, check for unclosed tags |
| Code example hangs | Ensure `rclpy.shutdown()` is called |
| Diagram misaligned | Use monospace font preview, check character width |

---

## File Checklist

### Required Deliverables

```
my-website/
├── docs/
│   └── module-1-ros2-fundamentals/
│       ├── _category_.json        [ ]
│       ├── chapter-1-ros2-overview.mdx    [ ] 1,200-1,400 words
│       ├── chapter-2-rclpy-nodes.mdx      [ ] 1,400-1,800 words
│       └── chapter-3-urdf-basics.mdx      [ ] 1,200-1,600 words
└── static/
    └── code-examples/
        └── module-1/
            ├── publisher.py       [ ] Tested
            ├── subscriber.py      [ ] Tested
            ├── service_server.py  [ ] Tested
            ├── service_client.py  [ ] Tested
            └── humanoid_torso.urdf [ ] Validated
```

### Verification Sign-off

| Check | Passed | Date | Verifier |
|-------|--------|------|----------|
| All code executes | [ ] | | |
| URDF validates | [ ] | | |
| Word counts in range | [ ] | | |
| Docusaurus builds | [ ] | | |
| Technical review | [ ] | | |
