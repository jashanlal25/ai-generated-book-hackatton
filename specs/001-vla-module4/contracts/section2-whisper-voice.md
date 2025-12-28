# Contract: Section 2 - Voice-to-Action with Whisper

## Metadata
- **Word Target**: 500 words
- **Priority**: P1
- **Dependencies**: Section 1

## Learning Objectives
1. Understand how speech recognition enables human-robot interaction
2. Describe Whisper's role in the voice-to-action pipeline
3. Outline ROS 2 integration patterns for voice commands

## Required Content

### 2.1 Speech Recognition for Robotics
- Why voice input: Natural human-robot interaction
- Challenges: Background noise, accents, real-time processing
- Solution: Modern transformer-based ASR (Whisper)

### 2.2 Whisper Architecture Overview
- Encoder-decoder transformer for speech-to-text
- Model sizes: tiny (39M) to large (1.5B) - tradeoff: latency vs accuracy
- Multilingual capability; robust to noise
- Audio processing: 16kHz, 30-second chunks

### 2.3 ROS 2 Integration Pattern
- Audio capture node: Microphone → audio topic
- Whisper node: Audio → text transcription
- Command publisher: Text → /voice_command topic
- Downstream: VLA planner consumes voice commands

### 2.4 Confidence Filtering
- Why filter: Prevent misrecognized commands from executing
- Threshold: Only pass commands with confidence > 0.8
- Fallback: "Did you mean...?" clarification

## Required Diagram
Voice command pipeline from microphone to ROS 2 topic.

## Required Citations
- Radford et al. (2023) - Whisper paper
- OpenAI Whisper documentation

## Acceptance Criteria
- [ ] Explains Whisper's role in VLA pipeline
- [ ] Describes model size tradeoffs
- [ ] Outlines ROS 2 integration pattern
- [ ] Mentions confidence filtering
- [ ] APA citations present
