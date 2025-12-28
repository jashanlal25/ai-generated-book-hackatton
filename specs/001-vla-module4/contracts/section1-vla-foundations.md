# Contract: Section 1 - VLA Foundations

## Metadata
- **Word Target**: 400 words
- **Priority**: P1
- **Dependencies**: None

## Learning Objectives
1. Define Vision-Language-Action (VLA) systems
2. Identify the three core components: vision, language, action
3. Understand how these components integrate into a unified pipeline

## Required Content

### 1.1 What are VLA Systems?
- Definition: Multimodal models that process images + text → robot actions
- Historical context: Evolution from separate vision/NLP/control systems
- Key insight: LLMs can learn action representations as "another language"

### 1.2 The Three-Stage Architecture
- **Vision Encoder**: ResNet/ViT extracts spatial features from RGB
- **Language Model**: Processes text + image embeddings for reasoning
- **Action Decoder**: Maps embeddings to motor commands (joint angles, velocities)

### 1.3 Key VLA Models
- RT-2 (Google): Actions as text tokens, 55B parameters
- OpenVLA (Stanford): Open-source, 7B parameters, outperforms RT-2-X
- Brief mention: Octo (diffusion-based), GR00T N1 (humanoid-specific)

## Required Diagram
Three-stage VLA pipeline showing data flow from image input to motor output.

## Required Citations
- Brohan et al. (2023) - RT-2
- Kim et al. (2024) - OpenVLA

## Acceptance Criteria
- [ ] Defines VLA clearly for intermediate readers
- [ ] Explains three-stage architecture
- [ ] Includes at least 2 model examples
- [ ] APA citations present
