---
id: 0001
title: Create Project Constitution
stage: constitution
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- /mnt/e/Hackathon_wsl_project_2025/.specify/memory/constitution.md
tests:
- none
---

## Prompt

```text
/sp.constitution

Project: Docusaurus book + integrated RAG chatbot on Physical AI & Humanoid Robotics.

Goals:
- Produce accurate, citation-backed technical content.
- Explain AI systems controlling humanoid robots using ROS 2, Gazebo, Unity, NVIDIA Isaac, VSLAM, Nav2, and VLA.
- Provide deployable code and reproducible workflows.
- Document and implement a complete RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant Cloud.

Core Principles:
1. Accuracy: All technical claims must match official docs or peer-reviewed sources.
2. Clarity: Audience: robotics/CS students; concise, structured explanations.
3. Reproducibility: Code must be functional; architectures must be implementable.
4. No hallucinations: Only verifiable statements allowed.

Standards:
- Citations: APA.
- Split sources: ≥50% peer-reviewed, rest from official docs.
- Zero plagiarism.
- Reading level: FK grade 11–13.
- Diagrams described textually for Docusaurus compatibility.

Book Requirements:
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

RAG Requirements:
- Must include architecture, example API, embedding pipeline, database schema, text-selection mode, and deployment steps.
- Content must be production-grade.

Constraints:
- Book length: 25k–40k words.
- ≥50 sources.
- Include GitHub Pages deployment instructions.

Success Criteria:
- Correct robotics/AI content.
- Executable code.
- Accurate citations.
- Complete RAG implementation docs.
- Fully buildable book and functional chatbot.

/sp.end
```

## Response snapshot

The project constitution for 'Docusaurus book + integrated RAG chatbot on Physical AI & Humanoid Robotics' has been created and updated to version 1.0.0. Core principles, standards, project requirements, constraints, and success criteria are now documented. All dependent templates have been reviewed for consistency.

## Outcome

- ✅ Impact: Initial project constitution established, defining core principles, standards, and requirements for the Docusaurus book and RAG chatbot.
- 🧪 Tests: none
- 📁 Files: 1 file modified: /mnt/e/Hackathon_wsl_project_2025/.specify/memory/constitution.md
- 🔁 Next prompts: User to review the constitution and initiate further spec/plan commands.
- 🧠 Reflection: Successfully parsed user intent and populated constitution. Fallback PHR creation handled due to script path issue.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script failed to execute due to an incorrect relative path. Manual PHR creation using the template was initiated.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure all script paths are absolute or correctly relative from the current working directory to avoid execution failures.
