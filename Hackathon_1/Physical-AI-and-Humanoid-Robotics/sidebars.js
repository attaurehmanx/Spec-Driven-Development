// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Manual sidebar configuration for the textbook
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI',
      items: [
        {
          type: 'category',
          label: 'Chapter 1',
          items: [
            'robotics/module1/chapter1/lesson1',
            'robotics/module1/chapter1/lesson2',
            'robotics/module1/chapter1/lesson3',
            'robotics/module1/chapter1/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2',
          items: [
            'robotics/module1/chapter2/lesson1',
            'robotics/module1/chapter2/lesson2',
            'robotics/module1/chapter2/lesson3',
            'robotics/module1/chapter2/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3',
          items: [
            'robotics/module1/chapter3/lesson1',
            'robotics/module1/chapter3/lesson2',
            'robotics/module1/chapter3/lesson3',
            'robotics/module1/chapter3/lesson4',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Foundations',
      items: [
        {
          type: 'category',
          label: 'Chapter 1',
          items: [
            'robotics/module2/chapter1/lesson1',
            'robotics/module2/chapter1/lesson2',
            'robotics/module2/chapter1/lesson3',
            'robotics/module2/chapter1/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2',
          items: [
            'robotics/module2/chapter2/lesson1',
            'robotics/module2/chapter2/lesson2',
            'robotics/module2/chapter2/lesson3',
            'robotics/module2/chapter2/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3',
          items: [
            'robotics/module2/chapter3/lesson1',
            'robotics/module2/chapter3/lesson2',
            'robotics/module2/chapter3/lesson3',
            'robotics/module2/chapter3/lesson4',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac AI Brain - Advanced Perception, Mapping, and Navigation',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: AI Perception Fundamentals',
          items: [
            'robotics/module3/chapter1/lesson1',
            'robotics/module3/chapter1/lesson2',
            'robotics/module3/chapter1/lesson3',
            'robotics/module3/chapter1/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Mapping and Localization',
          items: [
            'robotics/module3/chapter2/lesson1',
            'robotics/module3/chapter2/lesson2',
            'robotics/module3/chapter2/lesson3',
            'robotics/module3/chapter2/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Navigation and Path Planning',
          items: [
            'robotics/module3/chapter3/lesson1',
            'robotics/module3/chapter3/lesson2',
            'robotics/module3/chapter3/lesson3',
            'robotics/module3/chapter3/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Isaac Sim Topics',
          items: [
            'robotics/module3/isaac-sim/introduction',
            'robotics/module3/isaac-sim/synthetic-data-generation',
            'robotics/module3/isaac-sim/photorealistic-rendering',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) for Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Vision & Perception Systems',
          items: [
            'robotics/module4/chapter1/lesson1-introduction-to-computer-vision',
            'robotics/module4/chapter1/lesson2-object-detection-isac-sim',
            'robotics/module4/chapter1/lesson3-environmental-mapping',
            'robotics/module4/chapter1/lesson4-perception-integration',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Language Processing & Planning',
          items: [
            'robotics/module4/chapter2/lesson1',
            'robotics/module4/chapter2/lesson2',
            'robotics/module4/chapter2/lesson3',
            'robotics/module4/chapter2/lesson4',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Action & Execution',
          items: [
            'robotics/module4/chapter3/lesson1',
            'robotics/module4/chapter3/lesson2',
            'robotics/module4/chapter3/lesson3',
            'robotics/module4/chapter3/lesson4',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
