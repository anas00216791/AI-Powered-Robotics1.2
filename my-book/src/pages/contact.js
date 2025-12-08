import React from 'react';
import Layout from '@theme/Layout';
import styles from './contact.module.css';

export default function Contact() {
  return (
    <Layout
      title="Contact"
      description="Contact AI-Powered Robotics Course">
      <div className={styles.contactContainer}>
        <div className="container">
          <h1 className={styles.pageTitle}>Get In Touch</h1>

          <div className={styles.introText}>
            <p>
              Have questions about the course? Want to provide feedback or report an issue?
              We'd love to hear from you!
            </p>
          </div>

          <div className={styles.contactGrid}>
            <div className={styles.contactCard}>
              <div className={styles.cardIcon}>📧</div>
              <h3>Email Us</h3>
              <p>For course-related questions and support</p>
              <a href="mailto:course@example.com" className={styles.contactLink}>
                tariqanas825@gmail.com
              </a>
            </div>

            <div className={styles.contactCard}>
              <div className={styles.cardIcon}>💬</div>
              <h3>Community Forum</h3>
              <p>Join discussions with other learners</p>
              <a href="#" className={styles.contactLink}>
                Visit Forum →
              </a>
            </div>

            <div className={styles.contactCard}>
              <div className={styles.cardIcon}>🐛</div>
              <h3>Report Issues</h3>
              <p>Found a bug or error in the content?</p>
              <a href="https://github.com/anas00216791" target="_blank" rel="noopener noreferrer" className={styles.contactLink}>
                GitHub Issues →
              </a>
            </div>

            <div className={styles.contactCard}>
              <div className={styles.cardIcon}>💡</div>
              <h3>Suggestions</h3>
              <p>Have ideas to improve the course?</p>
              <a href="#" className={styles.contactLink}>
                Share Your Ideas →
              </a>
            </div>
          </div>

          <section className={styles.faqSection}>
            <h2>Frequently Asked Questions</h2>

            <div className={styles.faqItem}>
              <h3>How long does it take to complete the course?</h3>
              <p>
                The complete course takes approximately 6-8 weeks if you dedicate 10-15 hours per week.
                However, you can learn at your own pace.
              </p>
            </div>

            <div className={styles.faqItem}>
              <h3>What are the prerequisites?</h3>
              <p>
                Basic Python programming knowledge, understanding of 3D coordinates, and familiarity with
                command line interface. Machine learning experience is helpful but not required.
              </p>
            </div>

            <div className={styles.faqItem}>
              <h3>Do I need special hardware?</h3>
              <p>
                A modern computer with at least 16GB RAM is recommended. For Module 3 (Isaac Sim),
                an NVIDIA GPU is highly recommended but not strictly required for learning.
              </p>
            </div>

            <div className={styles.faqItem}>
              <h3>Is there a certificate upon completion?</h3>
              <p>
                Currently, this is an open educational resource. We're working on adding completion
                certificates in the future.
              </p>
            </div>

            <div className={styles.faqItem}>
              <h3>Can I contribute to the course?</h3>
              <p>
                Yes! This is an open-source project. You can contribute by submitting improvements,
                reporting issues, or suggesting new content on our GitHub repository.
              </p>
            </div>
          </section>

          <section className={styles.supportSection}>
            <h2>Need Help?</h2>
            <p>
              Before reaching out, please check:
            </p>
            <ul className={styles.supportList}>
              <li>📚 The course documentation and module content</li>
              <li>❓ The FAQ section above</li>
              <li>💬 Community forum for similar questions</li>
              <li>🔍 Search existing GitHub issues</li>
            </ul>
            <p>
              If you still can't find an answer, feel free to contact us using any of the methods above.
              We typically respond within 24-48 hours.
            </p>
          </section>

          <div className={styles.socialSection}>
            <h2>Connect With Us</h2>
            <div className={styles.socialLinks}>
              <a href="#" className={styles.socialButton}>
                GitHub
              </a>
              <a href="#" className={styles.socialButton}>
                Twitter
              </a>
              <a href="#" className={styles.socialButton}>
                LinkedIn
              </a>
              <a href="#" className={styles.socialButton}>
                Discord
              </a>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
