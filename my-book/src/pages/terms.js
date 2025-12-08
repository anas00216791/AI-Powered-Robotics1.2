import React from 'react';
import Layout from '@theme/Layout';
import styles from './legal.module.css';

export default function Terms() {
  return (
    <Layout
      title="Terms of Service"
      description="Terms of Service for AI-Powered Robotics Course">
      <div className={styles.legalContainer}>
        <div className={styles.legalContent}>
          <h1>Terms of Service</h1>
          <p className={styles.lastUpdated}>Last Updated: December 9, 2025</p>

          <section className={styles.section}>
            <h2>1. Acceptance of Terms</h2>
            <p>
              By accessing and using the AI-Powered Robotics Course platform, you accept and agree to be bound by the terms and provision of this agreement.
            </p>
          </section>

          <section className={styles.section}>
            <h2>2. Use License</h2>
            <p>
              Permission is granted to temporarily access the materials (information or software) on AI-Powered Robotics Course for personal, non-commercial transitory viewing only.
            </p>
            <p>This is the grant of a license, not a transfer of title, and under this license you may not:</p>
            <ul>
              <li>Modify or copy the materials</li>
              <li>Use the materials for any commercial purpose</li>
              <li>Attempt to decompile or reverse engineer any software contained on the platform</li>
              <li>Remove any copyright or other proprietary notations from the materials</li>
              <li>Transfer the materials to another person or "mirror" the materials on any other server</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>3. User Accounts</h2>
            <p>
              To access certain features of the platform, you must register for an account. You agree to provide accurate, current, and complete information during the registration process and to update such information to keep it accurate, current, and complete.
            </p>
            <p>
              You are responsible for safeguarding your password and for all activities that occur under your account.
            </p>
          </section>

          <section className={styles.section}>
            <h2>4. Course Content</h2>
            <p>
              All course materials, including but not limited to videos, text, images, code, and documentation, are the intellectual property of AI-Powered Robotics Course or its content creators.
            </p>
            <p>
              You may not reproduce, distribute, modify, create derivative works of, publicly display, publicly perform, republish, download, store, or transmit any of the material on our platform without prior written consent.
            </p>
          </section>

          <section className={styles.section}>
            <h2>5. Code of Conduct</h2>
            <p>You agree to use the platform in accordance with all applicable laws and regulations. You agree not to:</p>
            <ul>
              <li>Use the platform for any unlawful purpose</li>
              <li>Harass, abuse, or harm other users</li>
              <li>Upload malicious code or viruses</li>
              <li>Attempt to gain unauthorized access to the platform or other users' accounts</li>
              <li>Interfere with or disrupt the platform's servers or networks</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>6. Third-Party Services</h2>
            <p>
              Our platform may contain links to third-party websites or services that are not owned or controlled by AI-Powered Robotics Course. We have no control over, and assume no responsibility for, the content, privacy policies, or practices of any third-party websites or services.
            </p>
          </section>

          <section className={styles.section}>
            <h2>7. Disclaimer</h2>
            <p>
              The materials on AI-Powered Robotics Course platform are provided on an 'as is' basis. We make no warranties, expressed or implied, and hereby disclaim and negate all other warranties including, without limitation, implied warranties or conditions of merchantability, fitness for a particular purpose, or non-infringement of intellectual property or other violation of rights.
            </p>
          </section>

          <section className={styles.section}>
            <h2>8. Limitations</h2>
            <p>
              In no event shall AI-Powered Robotics Course or its suppliers be liable for any damages (including, without limitation, damages for loss of data or profit, or due to business interruption) arising out of the use or inability to use the materials on the platform.
            </p>
          </section>

          <section className={styles.section}>
            <h2>9. Modifications</h2>
            <p>
              AI-Powered Robotics Course may revise these terms of service at any time without notice. By using this platform, you are agreeing to be bound by the then-current version of these terms of service.
            </p>
          </section>

          <section className={styles.section}>
            <h2>10. Governing Law</h2>
            <p>
              These terms and conditions are governed by and construed in accordance with applicable laws, and you irrevocably submit to the exclusive jurisdiction of the courts in that location.
            </p>
          </section>

          <section className={styles.section}>
            <h2>Contact Information</h2>
            <p>
              If you have any questions about these Terms of Service, please contact us at:{' '}
              <a href="mailto:support@ai-robotics-course.com">support@ai-robotics-course.com</a>
            </p>
          </section>
        </div>
      </div>
    </Layout>
  );
}
