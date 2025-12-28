<!--
Sync Impact Report:
- Version change: N/A -> 1.0.0
- List of modified principles:
  - Added: Accuracy
  - Added: Clarity
  - Added: Reproducibility
  - Added: No Hallucinations
- Added sections: Core Principles, Standards, Project Requirements, Governance
- Removed sections: None
- Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending)
  - .specify/templates/spec-template.md (⚠ pending)
  - .specify/templates/tasks-template.md (⚠ pending)
  - .claude/commands/sp.adr.md (⚠ pending)
  - .claude/commands/sp.analyze.md (⚠ pending)
  - .claude/commands/sp.checklist.md (⚠ pending)
  - .claude/commands/sp.clarify.md (⚠ pending)
  - .claude/commands/sp.constitution.md (⚠ pending)
  - .claude/commands/sp.git.commit_pr.md (⚠ pending)
  - .claude/commands/sp.implement.md (⚠ pending)
  - .claude/commands/sp.phr.md (⚠ pending)
  - .claude/commands/sp.plan.md (⚠ pending)
  - .claude/commands/sp.specify.md (⚠ pending)
  - .claude/commands/sp.tasks.md (⚠ pending)
- Follow-up TODOs: None
-->
# Docusaurus book + integrated RAG chatbot on Physical AI & Humanoid Robotics Constitution

## Core Principles

### 1. Accuracy
All technical claims must match official docs or peer-reviewed sources.

### 2. Clarity
Audience: robotics/CS students; concise, structured explanations.

### 3. Reproducibility
Code must be functional; architectures must be implementable.

### 4. No Hallucinations
Only verifiable statements allowed.

## Standards

### Content & Citations
- Citations: APA.
- Split sources: ≥50% peer-reviewed, rest from official docs.
- Zero plagiarism.
- Reading level: FK grade 11–13.
- Diagrams described textually for Docusaurus compatibility.

## Project Requirements

### Book Requirements
- Format: Markdown/MDX.
- Include tutorials, examples, setups, troubleshooting, and chapter summaries.
- Required parts:
  1. Physical AI fundamentals
  2. ROS 2 (nodes, topics, services, URDF, Python bridges)
  3. Simulation: Gazebo (physics, sensors) + Unity digital twins
  4. NVIDIA Isaac (Isaac Sim, Isaac ROS, VSLAM, Nav2)
  5. VLA systems (Whisper, LLM planning to ROS actions)
  6. RAG chatbot architecture + implementation
  7. Capstone: Autonomous humanoid pipeline end-to-end

### RAG Requirements
- Must include architecture, example API, embedding pipeline, database schema, text-selection mode, and deployment steps.
- Content must be production-grade.

### Constraints
- Book length: 25k–40k words.
- ≥50 sources.
- Include GitHub Pages deployment instructions.

### Success Criteria
- Correct robotics/AI content.
- Executable code.
- Accurate citations.
- Complete RAG implementation docs.
- Fully buildable book and functional chatbot.

## Governance
This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All Pull Requests and reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08
