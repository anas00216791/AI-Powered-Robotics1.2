import React from 'react';
import Layout from '@theme/Layout';
import styles from './about.module.css';

export default function About() {
  return (
    <Layout
      title="About"
      description="About AI-Powered Robotics Course">
      <div className={styles.aboutContainer}>
        <div className="container">
          <h1 className={styles.pageTitle}>About This Course</h1>

          <section className={styles.section}>
            <h2>Our Mission</h2>
            <p>
              This course is designed to bridge the gap between traditional robotics education
              and cutting-edge AI-powered robotic systems. We aim to provide a comprehensive,
              hands-on learning experience that prepares students for the future of robotics.
            </p>
          </section>

          <section className={styles.section}>
            <h2>What Makes This Course Unique</h2>
            <div className={styles.features}>
              <div className={styles.feature}>
                <h3>🎯 Practical Focus</h3>
                <p>Every concept is taught with working code examples and hands-on exercises.</p>
              </div>
              <div className={styles.feature}>
                <h3>🔄 Complete Pipeline</h3>
                <p>From basic ROS 2 to advanced VLA models - the full robotics stack.</p>
              </div>
              <div className={styles.feature}>
                <h3>🚀 Industry-Relevant</h3>
                <p>Using professional tools: ROS 2, Gazebo, Unity, and NVIDIA Isaac Sim.</p>
              </div>
              <div className={styles.feature}>
                <h3>🧠 AI-First Approach</h3>
                <p>Emphasis on modern AI techniques including vision-language-action models.</p>
              </div>
            </div>
          </section>

          <section className={styles.section}>
            <h2>Course Philosophy</h2>
            <p>
              We believe in learning by building. Each module combines:
            </p>
            <ul className={styles.philosophyList}>
              <li><strong>Theory:</strong> Understanding the "why" behind each concept</li>
              <li><strong>Practice:</strong> Implementing solutions with real code</li>
              <li><strong>Simulation:</strong> Testing in safe, virtual environments</li>
              <li><strong>Integration:</strong> Building complete, working systems</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>Who Should Take This Course</h2>
            <div className={styles.audienceGrid}>
              <div className={styles.audienceCard}>
                <h4>🎓 Students</h4>
                <p>University students studying robotics, AI, or computer science</p>
              </div>
              <div className={styles.audienceCard}>
                <h4>💼 Professionals</h4>
                <p>Engineers transitioning into robotics and AI</p>
              </div>
              <div className={styles.audienceCard}>
                <h4>🔬 Researchers</h4>
                <p>Researchers exploring robot learning and embodied AI</p>
              </div>
              <div className={styles.audienceCard}>
                <h4>🌟 Self-Learners</h4>
                <p>Self-motivated individuals entering the field</p>
              </div>
            </div>
          </section>

          <section className={styles.section}>
            <h2>Learning Outcomes</h2>
            <p>By completing this course, you will be able to:</p>
            <ul className={styles.outcomesList}>
              <li>Design and implement ROS 2 applications using Python</li>
              <li>Create and simulate robots in multiple platforms (Gazebo, Unity, Isaac Sim)</li>
              <li>Generate synthetic training data for machine learning models</li>
              <li>Deploy vision-language-action models for robot control</li>
              <li>Architect complete intelligent robotic systems from scratch</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>Technology Stack</h2>
            <div className={styles.techStack}>
              <div className={styles.techCategory}>
                <h4>Middleware</h4>
                <p>ROS 2 (Robot Operating System 2)</p>
              </div>
              <div className={styles.techCategory}>
                <h4>Simulation</h4>
                <p>Gazebo, Unity, NVIDIA Isaac Sim</p>
              </div>
              <div className={styles.techCategory}>
                <h4>Programming</h4>
                <p>Python, rclpy, PyTorch</p>
              </div>
              <div className={styles.techCategory}>
                <h4>AI Models</h4>
                <p>VLA Models (RT-1, RT-2, OpenVLA)</p>
              </div>
            </div>
          </section>

          <section className={styles.section}>
            <h2>Course Development</h2>
            <p>
              This course is continuously updated to reflect the latest developments in
              robotics and AI. We incorporate feedback from students and industry professionals
              to ensure the content remains relevant and practical.
            </p>
            <p className={styles.updateInfo}>
              <strong>Last Updated:</strong> December 2025
            </p>
          </section>
        </div>
      </div>
    </Layout>
  );
}
