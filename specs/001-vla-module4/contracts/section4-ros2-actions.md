# Contract: Section 4 - ROS 2 Action Sequencing

## Metadata
- **Word Target**: 600 words
- **Priority**: P2
- **Dependencies**: Section 3, Module 1 ROS 2 knowledge

## Learning Objectives
1. Map LLM-generated plans to ROS 2 action interfaces
2. Identify Nav2, MoveIt, and custom action servers
3. Understand Behavior Trees for task orchestration

## Required Content

### 4.1 From Plan to Execution
- Bridge: LLM outputs JSON → ROS 2 executor dispatches actions
- Pattern: Action clients send goals to action servers
- Asynchronous: Non-blocking execution with feedback/result callbacks

### 4.2 Action Server Mapping

| Action Type | ROS 2 Server | Message Type |
|-------------|--------------|--------------|
| navigate | Nav2 | NavigateToPose |
| detect | Perception server | DetectObjects |
| grasp | MoveIt 2 | MoveGroupAction |
| place | MoveIt 2 | MoveGroupAction |

### 4.3 Action Client Pattern
- Create client for each action type
- Wait for server availability
- Send goal asynchronously
- Handle feedback during execution
- Process result (success/failure)

### 4.4 Behavior Trees for Sequencing
- Why BTs: More flexible than state machines
- BT.ROS2: Standard ROS 2 integration
- Nav2 uses BTs internally for navigation
- Sequence nodes: Execute children in order
- Fallback nodes: Try alternatives on failure

### 4.5 Error Handling
- Navigation failure: Replan with intermediate waypoints
- Detection failure: Move to alternative viewpoint
- Grasp failure: Try different grasp type
- Ambiguity: Query human for clarification

## Required Diagram
ROS 2 action executor showing plan dispatch to multiple action clients.

## Required Citations
- Macenski et al. (2020) - Nav2 / Marathon 2
- ROS 2 Action documentation
- MoveIt 2 documentation

## Acceptance Criteria
- [ ] Maps plan actions to ROS 2 interfaces
- [ ] Explains action client pattern
- [ ] Introduces Behavior Trees
- [ ] Covers error handling strategies
- [ ] APA citations present
