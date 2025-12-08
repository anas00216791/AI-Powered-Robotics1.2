import React, { useState, useEffect } from 'react';
import styles from './WelcomeModal.module.css';

export default function WelcomeModal() {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const hasVisited = localStorage.getItem('hasVisitedBefore');
    if (!hasVisited) {
      setTimeout(() => setIsVisible(true), 1000);
    }
  }, []);

  const handleClose = () => {
    setIsVisible(false);
    localStorage.setItem('hasVisitedBefore', 'true');
  };

  const handleGetStarted = () => {
    localStorage.setItem('hasVisitedBefore', 'true');
    setIsVisible(false);
    window.location.href = '/intro';
  };

  if (!isVisible) return null;

  return (
    <div className={styles.modalOverlay} onClick={handleClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={handleClose}>×</button>

        <div className={styles.modalHeader}>
          <div className={styles.robotIcon}>🤖</div>
          <h2 className={styles.modalTitle}>Welcome to AI-Powered Robotics!</h2>
          <p className={styles.modalSubtitle}>Your Journey to Building Intelligent Machines Starts Here</p>
        </div>

        <div className={styles.modalBody}>
          <div className={styles.features}>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>🚀</span>
              <h4>4 Comprehensive Modules</h4>
              <p>From ROS 2 basics to VLA models</p>
            </div>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>💻</span>
              <h4>Hands-on Learning</h4>
              <p>Real code examples and exercises</p>
            </div>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>🎓</span>
              <h4>Professional Tools</h4>
              <p>Industry-standard platforms</p>
            </div>
          </div>

          <div className={styles.stats}>
            <div className={styles.stat}>
              <div className={styles.statNumber}>6-8</div>
              <div className={styles.statLabel}>Weeks</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statNumber}>12+</div>
              <div className={styles.statLabel}>Chapters</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statNumber}>100%</div>
              <div className={styles.statLabel}>Free</div>
            </div>
          </div>
        </div>

        <div className={styles.modalFooter}>
          <button className={styles.primaryButton} onClick={handleGetStarted}>
            Start Learning Now
          </button>
          <button className={styles.secondaryButton} onClick={handleClose}>
            Explore First
          </button>
        </div>
      </div>
    </div>
  );
}
