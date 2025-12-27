import React, { useState } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import { motion } from 'framer-motion';
import styles from './styles.module.css';

// Define the 4 core modules for the platform
const ModuleList = [
  {
    id: 'physical-ai',
    title: 'Physical AI Fundamentals',
    icon: 'fas fa-brain',
    description: (
      <>
        Master the core principles of embodied intelligence, sensorimotor learning,
        and real-world AI applications. Learn how to create intelligent systems
        that interact with the physical world.
      </>
    ),
    link: '/docs/robotics/module1/chapter1/lesson1',
  },
  {
    id: 'humanoid-robotics',
    title: 'Humanoid Robotics Systems',
    icon: 'fas fa-robot',
    description: (
      <>
        Explore advanced humanoid robot architectures, locomotion, balance control,
        and human-robot interaction. Build and program sophisticated bipedal robots.
      </>
    ),
    link: '/docs/robotics/module2/chapter1/lesson1',
  },
  {
    id: 'ros-integration',
    title: 'ROS 2 & System Integration',
    icon: 'fas fa-cogs',
    description: (
      <>
        Deep dive into ROS 2 frameworks, middleware communication, and distributed
        robotics systems. Master the tools used in professional robotics development.
      </>
    ),
    link: '/docs/robotics/module3/chapter1/lesson1',
  },
  {
    id: 'ai-robotics-applications',
    title: 'Applied AI Robotics',
    icon: 'fas fa-eye',
    description: (
      <>
        Implement real-world robotics solutions with computer vision, machine learning,
        and autonomous decision-making. Build robots that solve practical problems.
      </>
    ),
    link: '/docs/robotics/module4/chapter1/lesson1-introduction-to-computer-vision',
  },
];

function ModuleCard({ module, index }) {
  const [isHovered, setIsHovered] = useState(false);
  const [isPressed, setIsPressed] = useState(false);

  const handleClick = () => {
    // Navigate to the module's content page
    window.location.href = module.link;
  };

  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{
        duration: 0.5,
        delay: index * 0.1, // Staggered animation
        ease: "easeOut"
      }}
      whileHover={{
        y: -8,
        scale: 1.02,
        transition: { duration: 0.2 }
      }}
      whileTap={{
        scale: 0.98,
        transition: { duration: 0.1 }
      }}
      onHoverStart={() => setIsHovered(true)}
      onHoverEnd={() => setIsHovered(false)}
      onTapStart={() => setIsPressed(true)}
      onTap={() => setIsPressed(false)}
      onClick={handleClick}
      style={{
        cursor: 'pointer',
        transition: 'all 0.3s ease'
      }}
    >
      <div className={clsx(styles.featureCard, isHovered && styles.featureCardHovered, isPressed && styles.featureCardPressed)}>
        <div className="text--center padding-horiz--md">
          <div className={styles.moduleIcon}>
            <i className={module.icon + ' ' + styles.moduleIconSvg}></i>
          </div>
          <Heading as="h3" className={styles.featureTitle}>{module.title}</Heading>
          <p className={styles.featureDescription}>{module.description}</p>
          <div className={styles.moduleLink}>
            <Link
              to={module.link}
              className={styles.moduleLinkButton}
              onClick={(e) => {
                e.stopPropagation();
                window.location.href = module.link;
              }}
            >
              Explore Module →
            </Link>
          </div>
        </div>
      </div>
    </motion.div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={styles.sectionHeader}>
            <motion.div
              initial={{ opacity: 0, y: -20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.6, ease: "easeOut" }}
            >
              <Heading as="h2" className={styles.sectionTitle}>Core Learning Modules</Heading>
              <p className={styles.sectionSubtitle}>Comprehensive curriculum designed for next-generation robotics engineers</p>
            </motion.div>
          </div>
        </div>
        <div className={clsx('row', styles.modulesRow)}>
          {ModuleList.map((module, index) => (
            <ModuleCard key={module.id} module={module} index={index} />
          ))}
        </div>
      </div>
    </section>
  );
}
