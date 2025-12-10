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
  // Manual sidebar for the Physical AI & Humanoid Robotics textbook
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'About This Course',
      items: [
        'about/course-overview',
        'about/prerequisites',
        'about/learning-outcomes',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1',
        {
          type: 'category',
          label: 'Weeks 1-2: Introduction to Physical AI',
          items: [
            'module-1/week-1-2/introduction-to-physical-ai',
            'module-1/week-1-2/sensors-and-perception',
          ],
        },
        {
          type: 'category',
          label: 'Weeks 3-5: ROS 2 Fundamentals',
          items: [
            'module-1/week-3-5/ros2-architecture',
            'module-1/week-3-5/rclpy-packages',
            'module-1/week-3-5/nodes-topics-services',
            'module-1/week-3-5/publisher-subscriber-tutorial',
            'module-1/week-3-5/urdf-robot-description',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2',
        {
          type: 'category',
          label: 'Week 6: Gazebo Simulation',
          items: [
            'module-2/week-6/gazebo-simulation-basics',
            'module-2/week-6/urdf-sdf-robot-models',
          ],
        },
        {
          type: 'category',
          label: 'Week 7: Unity Integration',
          items: [
            'module-2/week-7/unity-integration',
            'module-2/week-7/multi-sensor-fusion',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3',
        {
          type: 'category',
          label: 'Week 8: NVIDIA Isaac Fundamentals',
          items: [
            'module-3/week-8/isaac-sim-basics',
          ],
        },
        {
          type: 'category',
          label: 'Week 9: Perception Systems',
          items: [
            'module-3/week-9/isaac-ros-integration',
            'module-3/week-9/perception-pipelines',
          ],
        },
        {
          type: 'category',
          label: 'Week 10: Navigation & Planning',
          items: [
            'module-3/week-10/navigation-planning',
            'module-3/week-10/sim-to-real-transfer',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4',
        {
          type: 'category',
          label: 'Week 11: Voice & Action Integration',
          items: [
            'module-4/week-11/voice-to-action',
            'module-4/week-11/whisper-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Embodied AI & Humanoid Control',
          items: [
            'module-4/week-12/embodied-ai-humanoid-control',
            'module-4/week-12/language-to-action',
            'module-4/week-12/llm-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: CAPSTONE PROJECT',
          items: [
            'module-4/week-13/capstone-project',
            'module-4/week-13/system-integration',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'resources/hardware-requirements',
        'resources/troubleshooting',
        'resources/further-reading',
      ],
    },
  ],
};

export default sidebars;
