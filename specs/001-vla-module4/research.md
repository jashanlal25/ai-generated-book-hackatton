# Research: Vision-Language-Action Systems (Module 4)

**Branch**: `001-vla-module4` | **Date**: 2025-12-09
**Purpose**: Resolve technical unknowns for VLA frameworks, Whisper integration, LLM planning, and ROS 2 action sequencing.

---

## 1. VLA Framework Landscape (Verified)

| Model | Organization | Year | Parameters | Key Innovation |
|-------|--------------|------|------------|----------------|
| RT-2 | Google DeepMind | 2023 | 55B | Actions as text tokens; web-scale pretraining |
| PaLM-E | Google | 2023 | 562B | End-to-end embodied reasoning |
| OpenVLA | Stanford/Berkeley | 2024 | 7B | Open-source; outperforms RT-2-X by 16.5% |
| Octo | UC Berkeley | 2024 | 93M | Diffusion-based actions; multi-embodiment |
| GR00T N1 | NVIDIA | 2025 | N/A | Humanoid-specific; dual-system architecture |

**Decision**: Focus on RT-2/OpenVLA architecture pattern for educational clarity.

**Rationale**: Well-documented, open-source options available, represents industry standard.

---

## 2. VLA Architecture Pattern (Verified)

### Three-Stage Pipeline

```text
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   VISION    │───>│  LANGUAGE   │───>│   ACTION    │
│   ENCODER   │    │    MODEL    │    │   DECODER   │
└─────────────┘    └─────────────┘    └─────────────┘
     │                   │                   │
     v                   v                   v
  RGB Image         Text Tokens         Motor Commands
  224×224+          + Image Embed       Joint angles,
                                        Velocities
```

**Key Papers**:
- Brohan et al. (2023). RT-2: Vision-language-action models. *CoRL*.
- Kim et al. (2024). OpenVLA: Open-source vision-language-action model. *arXiv*.

---

## 3. Whisper Speech Recognition (Verified)

| Model Size | Parameters | Latency | Use Case |
|------------|------------|---------|----------|
| Tiny | 39M | ~1-2s | Edge devices, real-time |
| Base | 74M | ~3-5s | Good balance |
| Small | 244M | ~5-10s | Higher accuracy |
| Large | 1.5B | ~10s+ | Best accuracy, GPU required |

**ROS 2 Integration Options**:
- `whisper_ros` (GitHub: mgonzs13/whisper_ros) - SileroVAD + whisper.cpp
- `ros2_whisper` (ros-ai) - Python inference node

**Key Paper**: Radford et al. (2023). Robust speech recognition via large-scale weak supervision. *ICML*.

---

## 4. LLM Planning for Robotics (Verified)

### Planning Approaches

| Approach | Description | Source |
|----------|-------------|--------|
| SayCan | LLM + affordance scoring | Ahn et al. (2022) |
| Code as Policies | LLM generates executable code | Liang et al. (2023) |
| ProgPrompt | Programmatic prompt templates | Singh et al. (2023) |
| SMART-LLM | Multi-robot task allocation | Kannan et al. (2023) |

**Recommended Pattern**: Two-stage planning
1. **Stage 1**: Frozen LLM generates structured action plan (JSON)
2. **Stage 2**: ROS 2 executor dispatches to action servers

**Key Papers**:
- Huang et al. (2023). Language models as zero-shot planners. *ICML*.
- Liang et al. (2023). Code as policies. *ICRA*.

---

## 5. ROS 2 Action Sequencing (Verified)

| Task Type | Action Server | Message Type |
|-----------|---------------|--------------|
| Navigation | Nav2 | `NavigateToPose` |
| Manipulation | MoveIt 2 | `MoveGroupAction` |
| Gripper | Custom | `GripperCommand` |
| Detection | Custom | `DetectObjects` |
| Behavior Seq | BT Navigator | `ExecuteTreeAction` |

**Best Practice**: Use Behavior Trees (BehaviorTree.ROS2) for task orchestration.

**Key Paper**: Macenski et al. (2020). The Marathon 2: A navigation system. *IROS*.

---

## 6. Simulation Environment (Verified)

| Platform | VLA Support | Humanoid Support |
|----------|-------------|------------------|
| Isaac Sim | GR00T N1, OpenVLA fine-tuning | 1X, Agility, Fourier |
| Isaac Lab | RL training workflows | Bipedal locomotion |

**Decision**: Reference Isaac Sim as simulation environment (continuity with Module 3).

---

## 7. Sources for APA Citations

### Peer-Reviewed (≥50%)

1. Brohan, A., et al. (2023). RT-2: Vision-language-action models. *Conference on Robot Learning*.
2. Kim, M., et al. (2024). OpenVLA: An open-source vision-language-action model. *arXiv:2406.09246*.
3. Radford, A., et al. (2023). Robust speech recognition via large-scale weak supervision. *ICML*.
4. Huang, W., et al. (2023). Language models as zero-shot planners. *ICML*.
5. Liang, J., et al. (2023). Code as policies. *ICRA*.
6. Macenski, S., et al. (2020). The Marathon 2: A navigation system. *IROS*.
7. Driess, D., et al. (2023). PaLM-E: An embodied multimodal language model. *ICML*.
8. Ahn, M., et al. (2022). Do as I can, not as I say: Grounding language in robotic affordances. *CoRL*.

### Official Documentation (≤50%)

9. OpenAI. (2024). *Whisper documentation*. https://github.com/openai/whisper
10. Open Robotics. (2024). *ROS 2 Humble documentation*. https://docs.ros.org/en/humble/
11. Open Robotics. (2024). *Nav2 documentation*. https://docs.nav2.org/
12. PickNik Robotics. (2024). *MoveIt 2 documentation*. https://moveit.picknik.ai/
13. NVIDIA. (2024). *Isaac Sim documentation*. https://docs.omniverse.nvidia.com/isaacsim/
14. Google DeepMind. (2023). *RT-2 project page*. https://robotics-transformer2.github.io/

---

## 8. Resolved Unknowns

| Unknown | Resolution | Confidence |
|---------|------------|------------|
| VLA architecture | Three-stage: vision→language→action | High |
| Speech recognition | Whisper (base/small for robotics) | High |
| LLM planning | Two-stage: frozen LLM + executor | High |
| ROS 2 integration | Action clients + Behavior Trees | High |
| Simulation | Isaac Sim (Module 3 continuity) | High |

**All research complete. Ready for Phase 1.**
