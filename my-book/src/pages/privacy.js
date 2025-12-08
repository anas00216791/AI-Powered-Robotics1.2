import React from 'react';
import Layout from '@theme/Layout';
import styles from './legal.module.css';

export default function Privacy() {
  return (
    <Layout
      title="Privacy Policy"
      description="Privacy Policy for AI-Powered Robotics Course">
      <div className={styles.legalContainer}>
        <div className={styles.legalContent}>
          <h1>Privacy Policy</h1>
          <p className={styles.lastUpdated}>Last Updated: December 9, 2025</p>

          <section className={styles.section}>
            <h2>1. Introduction</h2>
            <p>
              AI-Powered Robotics Course ("we," "our," or "us") is committed to protecting your privacy. This Privacy Policy explains how we collect, use, disclose, and safeguard your information when you use our platform.
            </p>
            <p>
              Please read this privacy policy carefully. If you do not agree with the terms of this privacy policy, please do not access the platform.
            </p>
          </section>

          <section className={styles.section}>
            <h2>2. Information We Collect</h2>

            <h3>2.1 Personal Information</h3>
            <p>We may collect personal information that you voluntarily provide to us when you:</p>
            <ul>
              <li>Register for an account</li>
              <li>Complete your profile</li>
              <li>Enroll in courses</li>
              <li>Participate in forums or discussions</li>
              <li>Contact us for support</li>
            </ul>
            <p>This information may include:</p>
            <ul>
              <li>Full name</li>
              <li>Email address</li>
              <li>Username and password</li>
              <li>Profile picture</li>
              <li>Learning preferences and interests</li>
            </ul>

            <h3>2.2 Automatically Collected Information</h3>
            <p>When you access our platform, we automatically collect certain information, including:</p>
            <ul>
              <li>IP address</li>
              <li>Browser type and version</li>
              <li>Device information</li>
              <li>Pages visited and time spent</li>
              <li>Referring website addresses</li>
              <li>Operating system</li>
            </ul>

            <h3>2.3 Course Progress and Performance Data</h3>
            <p>We collect information about your learning activities, including:</p>
            <ul>
              <li>Course enrollment and completion status</li>
              <li>Quiz and assignment scores</li>
              <li>Video watch time and progress</li>
              <li>Code submissions and projects</li>
              <li>Discussion forum participation</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>3. How We Use Your Information</h2>
            <p>We use the collected information for various purposes, including:</p>
            <ul>
              <li>Providing and maintaining the platform</li>
              <li>Creating and managing your account</li>
              <li>Personalizing your learning experience</li>
              <li>Tracking your course progress and performance</li>
              <li>Sending administrative information, updates, and notifications</li>
              <li>Responding to your inquiries and providing support</li>
              <li>Improving our platform and course content</li>
              <li>Analyzing usage patterns and trends</li>
              <li>Detecting and preventing fraud or abuse</li>
              <li>Complying with legal obligations</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>4. Cookies and Tracking Technologies</h2>
            <p>
              We use cookies, web beacons, and similar tracking technologies to collect and store information. Cookies are small files stored on your device that help us improve your experience.
            </p>
            <p>You can set your browser to refuse cookies or alert you when cookies are being sent. However, some parts of our platform may not function properly without cookies.</p>
          </section>

          <section className={styles.section}>
            <h2>5. Third-Party Services</h2>
            <p>We may use third-party services that collect, monitor, and analyze data to improve our platform:</p>
            <ul>
              <li>Analytics services (e.g., Google Analytics)</li>
              <li>Cloud hosting providers</li>
              <li>Payment processors</li>
              <li>Authentication services (OAuth providers)</li>
              <li>Email service providers</li>
            </ul>
            <p>These third parties have their own privacy policies and we have no responsibility or liability for their practices.</p>
          </section>

          <section className={styles.section}>
            <h2>6. Data Sharing and Disclosure</h2>
            <p>We do not sell, trade, or rent your personal information to third parties. We may share your information in the following situations:</p>
            <ul>
              <li><strong>With your consent:</strong> When you explicitly agree to share your information</li>
              <li><strong>Service providers:</strong> With trusted third-party service providers who assist us in operating our platform</li>
              <li><strong>Legal requirements:</strong> When required by law, subpoena, or other legal process</li>
              <li><strong>Business transfers:</strong> In connection with any merger, sale, or acquisition of all or a portion of our business</li>
              <li><strong>Protection of rights:</strong> To protect the rights, property, or safety of our users or the public</li>
            </ul>
          </section>

          <section className={styles.section}>
            <h2>7. Data Security</h2>
            <p>
              We implement appropriate technical and organizational security measures to protect your personal information against unauthorized access, alteration, disclosure, or destruction.
            </p>
            <p>These measures include:</p>
            <ul>
              <li>Encryption of data in transit and at rest</li>
              <li>Secure authentication mechanisms</li>
              <li>Regular security assessments</li>
              <li>Access controls and monitoring</li>
            </ul>
            <p>
              However, no method of transmission over the internet or electronic storage is 100% secure. While we strive to use commercially acceptable means to protect your information, we cannot guarantee absolute security.
            </p>
          </section>

          <section className={styles.section}>
            <h2>8. Data Retention</h2>
            <p>
              We retain your personal information for as long as necessary to provide our services and fulfill the purposes outlined in this Privacy Policy, unless a longer retention period is required or permitted by law.
            </p>
            <p>
              When we no longer need your information, we will securely delete or anonymize it.
            </p>
          </section>

          <section className={styles.section}>
            <h2>9. Your Privacy Rights</h2>
            <p>Depending on your location, you may have the following rights regarding your personal information:</p>
            <ul>
              <li><strong>Access:</strong> Request access to your personal information</li>
              <li><strong>Correction:</strong> Request correction of inaccurate or incomplete information</li>
              <li><strong>Deletion:</strong> Request deletion of your personal information</li>
              <li><strong>Portability:</strong> Request a copy of your data in a structured format</li>
              <li><strong>Objection:</strong> Object to the processing of your personal information</li>
              <li><strong>Restriction:</strong> Request restriction of processing your information</li>
              <li><strong>Withdraw consent:</strong> Withdraw your consent at any time</li>
            </ul>
            <p>To exercise these rights, please contact us using the information provided at the end of this policy.</p>
          </section>

          <section className={styles.section}>
            <h2>10. Children's Privacy</h2>
            <p>
              Our platform is not intended for children under the age of 13. We do not knowingly collect personal information from children under 13. If you are a parent or guardian and believe your child has provided us with personal information, please contact us, and we will delete such information.
            </p>
          </section>

          <section className={styles.section}>
            <h2>11. International Data Transfers</h2>
            <p>
              Your information may be transferred to and maintained on servers located outside of your country where data protection laws may differ. By using our platform, you consent to the transfer of your information to these locations.
            </p>
          </section>

          <section className={styles.section}>
            <h2>12. Changes to This Privacy Policy</h2>
            <p>
              We may update this Privacy Policy from time to time. We will notify you of any changes by posting the new Privacy Policy on this page and updating the "Last Updated" date.
            </p>
            <p>
              You are advised to review this Privacy Policy periodically for any changes. Changes are effective when posted on this page.
            </p>
          </section>

          <section className={styles.section}>
            <h2>13. Contact Us</h2>
            <p>
              If you have any questions, concerns, or requests regarding this Privacy Policy or our privacy practices, please contact us at:
            </p>
            <ul>
              <li>Email: <a href="mailto:privacy@ai-robotics-course.com">privacy@ai-robotics-course.com</a></li>
              <li>Support: <a href="mailto:support@ai-robotics-course.com">support@ai-robotics-course.com</a></li>
            </ul>
          </section>
        </div>
      </div>
    </Layout>
  );
}
