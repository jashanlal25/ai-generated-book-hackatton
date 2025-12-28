# Quickstart: Module 4 Content Authoring

## Prerequisites

- Docusaurus development environment
- Access to VLA research papers (RT-2, OpenVLA)
- Access to ROS 2 / Nav2 documentation
- Access to Whisper documentation

## File Creation Order

1. `my-website/docs/module-4-vla/_category_.json`
2. `my-website/docs/module-4-vla/intro.mdx`
3. `my-website/docs/module-4-vla/vla-foundations.mdx`
4. `my-website/docs/module-4-vla/whisper-voice-to-action.mdx`
5. `my-website/docs/module-4-vla/llm-cognitive-planning.mdx`
6. `my-website/docs/module-4-vla/ros2-action-sequencing.mdx`
7. `my-website/docs/module-4-vla/capstone-autonomous-humanoid.mdx`

## Content Guidelines

### Word Counts
- Total: 2,000-3,000 words
- Per section: See data-model.md

### Citation Format
```markdown
According to Brohan et al. (2023), RT-2 treats robot actions as text tokens...

## References

Brohan, A., Brown, N., Carbajal, J., & Zeng, A. (2023). RT-2: Vision-language-action models transfer web knowledge to robotic control. *Conference on Robot Learning*. https://robotics-transformer2.github.io/
```

### Diagram Format
Use ASCII/text diagrams within code blocks:
```markdown
```text
[Voice Input] --> [Whisper] --> [LLM] --> [Actions]
```
```

### Code Example Format (Minimal)
```markdown
```python
# Conceptual example - not production code
class VLAExecutor:
    async def execute_plan(self, plan):
        for action in plan:
            await self.dispatch(action)
```
```

## Validation Checklist

- [ ] Word count: 2,000-3,000
- [ ] APA citations present
- [ ] ≥50% peer-reviewed sources
- [ ] ≥3 concrete language-driven action examples
- [ ] 1 complete end-to-end pipeline diagram
- [ ] All technical terms defined
- [ ] FK readability grade 11-13
- [ ] Builds on Modules 1-3 knowledge
