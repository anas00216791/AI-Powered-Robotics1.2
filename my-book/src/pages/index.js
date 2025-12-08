import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import WelcomeModal from '../components/WelcomeModal';
import NewsletterPopup from '../components/NewsletterPopup';

function Hero() {
  return (
    <section className={styles.hero}>
      <div className="container">
        <h1 className={styles.heroTitle}>Welcome to AI-Powered Robotics</h1>
        <p className={styles.heroSubtitle}>
          Comprehensive learning from basics to advanced — Build intelligent machines
        </p>
        <div className={styles.heroButtons}>
          <Link
            className="button button--primary button--lg"
            to="/signup">
            Start Learning
          </Link>
        </div>
      </div>
    </section>
  );
}

function Introduction() {
  const benefits = [
    "Up-to-date knowledge in robotics and AI",
    "Hands-on and practical modules with real code",
    "Structured curriculum from ROS 2 to VLA models",
    "Career-oriented outcomes and skill growth in robotics"
  ];

  return (
    <section className={styles.introduction}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Why Study This Course?</h2>
        <ul className={styles.benefitsList}>
          {benefits.map((benefit, idx) => (
            <li key={idx} className={styles.benefitItem}>
              <span className={styles.checkmark}>✓</span>
              {benefit}
            </li>
          ))}
        </ul>
      </div>
    </section>
  );
}

function ModulesSection() {
  const modules = [
    {
      name: "Module 1: The Robotic Nervous System",
      title: "ROS 2 Basics",
      description: "Master robot middleware and communication. Learn ROS 2 architecture, Python integration, and robot modeling.",
      submodules: [
        "ROS 2 Basics - Nodes, Topics, and Services",
        "Python Agents with rclpy",
        "Humanoid URDF Modeling"
      ],
      link: "/module-1-ros2",
      icon: "🤖"
    },
    {
      name: "Module 2: The Digital Twin",
      title: "Gazebo & Unity Simulation",
      description: "Create realistic virtual environments for testing and training with physics simulation and photorealistic rendering.",
      submodules: [
        "Gazebo Physics Simulation",
        "Unity for High-Fidelity Rendering",
        "Sensor Simulation"
      ],
      link: "/module-2-gazebo-unity",
      icon: "🎮"
    },
    {
      name: "Module 3: NVIDIA Isaac Sim",
      title: "Professional-Grade Simulation",
      description: "Leverage GPU-accelerated simulation for AI training with photorealistic RTX rendering and synthetic data generation.",
      submodules: [
        "Getting Started with Isaac Sim",
        "ROS 2 Integration and Robot Control",
        "Synthetic Data Generation"
      ],
      link: "/module-3-isaac-sim",
      icon: "🌟"
    },
    {
      name: "Module 4: Vision-Language-Action Models",
      title: "The Future of Robot Control",
      description: "Control robots with natural language using foundation models. Learn end-to-end learning from vision to action.",
      submodules: [
        "Introduction to VLA Models",
        "Using Pre-trained VLA Models",
        "Fine-tuning and Deployment"
      ],
      link: "/module-4-vla",
      icon: "🧠"
    }
  ];

  return (
    <section className={styles.modules}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Curriculum & Topics</h2>
        <div className={styles.moduleGrid}>
          {modules.map((module, idx) => (
            <div key={idx} className={styles.moduleCard}>
              <div className={styles.moduleIcon}>{module.icon}</div>
              <h3 className={styles.moduleName}>{module.name}</h3>
              <h4 className={styles.moduleTitle}>{module.title}</h4>
              <p className={styles.moduleDescription}>{module.description}</p>
              <div className={styles.submodules}>
                <strong>Chapters:</strong>
                <ul>
                  {module.submodules.map((sub, subIdx) => (
                    <li key={subIdx}>{sub}</li>
                  ))}
                </ul>
              </div>
              <Link
                className="button button--secondary button--sm"
                to={module.link}>
                Start Module {idx + 1} →
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <div className={styles.homePage}>
      <WelcomeModal />
      <NewsletterPopup />
      <Hero />
      <main>
        <Introduction />
        <ModulesSection />
      </main>
    </div>
  );
}
