# Data Model: Module 4 - Vision-Language-Action Systems

**Date**: 2025-12-09 | **Plan**: [plan.md](./plan.md)

## Content Entities

### Primary Concepts

| Entity | Type | Definition | Relationships |
|--------|------|------------|---------------|
| VLA Model | Architecture | Multimodal model integrating vision, language, action | Contains: Vision Encoder, LLM, Action Decoder |
| Vision Encoder | Component | Extracts features from RGB images | Part of: VLA Model |
| Language Model | Component | Processes text instructions + image embeddings | Part of: VLA Model |
| Action Decoder | Component | Maps embeddings to motor commands | Part of: VLA Model |
| Whisper | Model | OpenAI speech-to-text transformer | Input to: LLM Planner |
| LLM Planner | Component | Decomposes commands into action plans | Outputs: Action Sequence |
| Action Vocabulary | Schema | Defined set of robot primitives | Used by: LLM Planner |
| ROS 2 Action Client | Interface | Goal-based control interface | Executes: Action Plan |
| Behavior Tree | Pattern | Task orchestration structure | Coordinates: Action Clients |

### Section-Entity Mapping

| Section | Primary Entities | Word Target |
|---------|-----------------|-------------|
| 1. VLA Foundations | VLA Model, Vision Encoder, Action Decoder | 400 |
| 2. Voice-to-Action | Whisper, Audio Pipeline | 500 |
| 3. LLM Planning | LLM Planner, Action Vocabulary | 600 |
| 4. ROS 2 Actions | Action Client, Behavior Tree | 600 |
| 5. Capstone | All entities combined | 400 |

## Diagram Specifications

### Diagram 1: VLA Three-Stage Pipeline
```
[RGB Image] --> [Vision Encoder] --> [Image Embeddings]
                                            |
                                            v
[Text Command] --> [Tokenizer] --> [Language Model] --> [Action Tokens]
                                            |
                                            v
                                    [Action Decoder] --> [Motor Commands]
```

### Diagram 2: Voice-to-Action Pipeline
```
[Microphone] --> [Audio Capture] --> [Whisper STT] --> [Text Command]
                                                              |
                                                              v
                                                       [LLM Planner]
                                                              |
                                                              v
                                                       [Action Plan]
```

### Diagram 3: LLM Task Decomposition
```
Input: "Fetch the water bottle from the table"
                    |
                    v
            [LLM Planner]
                    |
                    v
Output: [
  {"action": "navigate", "target": "table"},
  {"action": "detect", "object": "water_bottle"},
  {"action": "grasp", "object_id": "obj_123"},
  {"action": "navigate", "target": "user"}
]
```

### Diagram 4: ROS 2 Action Executor
```
[Action Plan] --> [VLA Executor Node]
                        |
        +---------------+---------------+
        |               |               |
        v               v               v
   [Nav2 Client]  [MoveIt Client]  [Gripper Client]
        |               |               |
        v               v               v
   [Navigate]      [Arm Motion]    [Grasp/Release]
```

### Diagram 5: Capstone End-to-End Flow
```
[Voice: "Bring me the book"]
        |
        v
[Whisper] --> [LLM Planner] --> [Action Sequence]
                                       |
        +------------------------------+
        |              |               |
        v              v               v
    [Nav2]       [Detection]      [MoveIt]
   Navigate      Find Book        Grasp Book
        |              |               |
        +------> [Behavior Tree] <-----+
                       |
                       v
               [Task Complete]
```

## Citation Requirements

| Claim Type | Required Source |
|------------|-----------------|
| VLA architecture | Brohan et al. (2023) |
| OpenVLA performance | Kim et al. (2024) |
| Whisper accuracy | Radford et al. (2023) |
| LLM planning | Huang et al. (2023) |
| Code as Policies | Liang et al. (2023) |
| Nav2 architecture | Macenski et al. (2020) |
| SayCan approach | Ahn et al. (2022) |
