# Contract: Section 3 - LLM Cognitive Planning

## Metadata
- **Word Target**: 600 words
- **Priority**: P1
- **Dependencies**: Section 2

## Learning Objectives
1. Understand how LLMs decompose natural language into robot actions
2. Learn prompt engineering patterns for robotic planning
3. Define action vocabularies for robot execution

## Required Content

### 3.1 LLMs as Robot Planners
- Key insight: LLMs have world knowledge useful for planning
- Approach: Use frozen LLM as semantic planner, not motor controller
- Benefits: Zero-shot generalization, natural language interface

### 3.2 Task Decomposition
- Example: "Fetch the water bottle" → [navigate, detect, grasp, return]
- Pattern: High-level intent → sequence of atomic actions
- State grounding: Include robot state in prompt context

### 3.3 Action Vocabulary
- Define available primitives: navigate, detect, grasp, place, etc.
- Structured output: JSON format for machine parsing
- Validation: Schema check before execution

### 3.4 Prompt Engineering Best Practices
- Few-shot examples: Show 2-3 exemplar decompositions
- System prompt: Define robot role and constraints
- State context: Current pose, detected objects, gripper state
- Output format: Force JSON with action sequence

### 3.5 Example Decompositions (≥3 required by spec)
1. "Pick up the red cup" → [detect(red_cup), navigate(cup_location), grasp(cup_id)]
2. "Put the book on the shelf" → [detect(book), grasp(book), navigate(shelf), place(book)]
3. "Bring me water" → [navigate(kitchen), detect(water_bottle), grasp(bottle), navigate(user), handover]

## Required Diagram
Task decomposition flow showing command → LLM → action sequence.

## Required Citations
- Huang et al. (2023) - LLMs as zero-shot planners
- Liang et al. (2023) - Code as Policies
- Ahn et al. (2022) - SayCan

## Acceptance Criteria
- [ ] Explains two-stage planning pattern
- [ ] Defines action vocabulary concept
- [ ] Includes ≥3 decomposition examples
- [ ] Covers prompt engineering basics
- [ ] APA citations present
