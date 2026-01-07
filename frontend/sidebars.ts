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
  // Textbook sidebar with modules and chapters
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      link: {
        type: 'doc',
        id: 'intro',
      },
    },
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI',
      items: [
        'introduction/what_is_physical_ai',
        'introduction/why_embodied_intelligence',
        'introduction/first_steps_with_ros',
        'introduction/lab_exercise_1',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS Fundamentals',
      items: [
        'ros_fundamentals/ros_architecture',
        'ros_fundamentals/nodes_topics_services',
        'ros_fundamentals/lab_exercise_1',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Gazebo Simulation',
      items: [
        'gazebo_simulation/introduction_to_gazebo',
        'gazebo_simulation/gazebo_ros_integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Unity Robotics',
      items: [
        'unity_robotics/unity_robotics_toolkit',
        'unity_robotics/unity_gym_environments',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: NVIDIA Isaac',
      items: [
        'nvidia_isaac/isaac_ros_framework',
        'nvidia_isaac/isaac_sim',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Vision-Language-Action Systems',
      items: [
        'vision_language_action/multimodal_learning',
        'vision_language_action/vla_robotics',
      ],
    },
    {
      type: 'category',
      label: 'Module 7: Humanoid Robotics',
      items: [
        'humanoid_robotics/humanoid_control',
        'humanoid_robotics/bipedal_locomotion',
      ],
    },
    {
      type: 'category',
      label: 'Module 8: Capstone - Autonomous Humanoid Project',
      items: [
        'capstone_project/project_overview',
        'capstone_project/implementation_guide',
      ],
    },
  ],
};

export default sidebars;
