# Contract: Section 5 - Capstone: Autonomous Humanoid

## Metadata
- **Word Target**: 400 words
- **Priority**: P3
- **Dependencies**: Sections 1-4

## Learning Objectives
1. Synthesize all VLA concepts in an end-to-end scenario
2. Trace a voice command through the complete pipeline
3. Identify failure points and recovery strategies

## Required Content

### 5.1 Capstone Scenario
- Command: "Bring me the book from the shelf"
- Robot: Humanoid with arms, mobile base, cameras
- Environment: Indoor room with shelf and user location

### 5.2 End-to-End Pipeline Walkthrough

1. **Voice Input**: User speaks command
2. **Whisper STT**: Audio → "bring me the book from the shelf"
3. **LLM Planning**: Text → action sequence
   - navigate(shelf)
   - detect(book)
   - grasp(book_id)
   - navigate(user)
   - handover(book)
4. **ROS 2 Execution**:
   - Nav2: Path to shelf
   - Perception: Object detection
   - MoveIt: Arm motion + grasp
   - Nav2: Return to user
   - Gripper: Release

### 5.3 Failure Points and Recovery
- Speech misrecognition: Confidence check + clarification
- Book not detected: Move viewpoint, retry
- Grasp fails: Alternative grasp pose
- Path blocked: Nav2 replanning

### 5.4 Complete Pipeline Diagram
Full end-to-end flow from voice to task completion.

### 5.5 Key Takeaways
- VLA integrates perception, reasoning, action
- Two-stage architecture: LLM planner + ROS executor
- Error handling is critical for robust autonomy
- Human-in-the-loop for ambiguity resolution

## Required Diagram
Complete capstone pipeline showing all stages from voice to completion.

## Required Citations
- Reference prior sections
- Module builds on Modules 1-3 foundations

## Acceptance Criteria
- [ ] Complete end-to-end scenario traced
- [ ] Pipeline diagram shows all stages
- [ ] Failure points identified with recovery
- [ ] Synthesizes all module concepts
- [ ] Provides clear takeaways
