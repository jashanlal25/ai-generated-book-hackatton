import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2-fundamentals/intro',
        'module-1-ros2-fundamentals/chapter-1-ros2-overview',
        'module-1-ros2-fundamentals/chapter-2-rclpy-nodes',
        'module-1-ros2-fundamentals/chapter-3-urdf-basics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/intro',
        'module-2-digital-twin/chapter-1-gazebo-physics',
        'module-2-digital-twin/chapter-2-unity-rendering',
        'module-2-digital-twin/chapter-3-sensor-simulation',
        'module-2-digital-twin/chapter-4-environment-building',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Brain (Isaac)',
      items: [
        'module-3-ai-brain/intro',
        'module-3-ai-brain/isaac-sim',
        'module-3-ai-brain/isaac-ros-vslam',
        'module-3-ai-brain/nav2-path-planning',
        'module-3-ai-brain/integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vla/intro',
        'module-4-vla/vla-foundations',
        'module-4-vla/whisper-voice-to-action',
        'module-4-vla/llm-cognitive-planning',
        'module-4-vla/ros2-action-sequencing',
        'module-4-vla/capstone-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
