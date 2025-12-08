import React, { useState, useEffect } from 'react';
import styles from './NewsletterPopup.module.css';

export default function NewsletterPopup() {
  const [isVisible, setIsVisible] = useState(false);
  const [email, setEmail] = useState('');
  const [isSubmitted, setIsSubmitted] = useState(false);

  useEffect(() => {
    const hasSubscribed = localStorage.getItem('newsletterSubscribed');
    const hasClosedNewsletter = localStorage.getItem('newsletterClosed');

    if (!hasSubscribed && !hasClosedNewsletter) {
      const timer = setTimeout(() => {
        setIsVisible(true);
      }, 15000); // Show after 15 seconds

      return () => clearTimeout(timer);
    }
  }, []);

  const handleClose = () => {
    setIsVisible(false);
    localStorage.setItem('newsletterClosed', 'true');
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (email) {
      setIsSubmitted(true);
      localStorage.setItem('newsletterSubscribed', 'true');
      setTimeout(() => {
        setIsVisible(false);
      }, 2000);
    }
  };

  if (!isVisible) return null;

  return (
    <div className={styles.popupOverlay} onClick={handleClose}>
      <div className={styles.popupContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={handleClose}>×</button>

        {!isSubmitted ? (
          <>
            <div className={styles.popupIcon}>📬</div>
            <h3 className={styles.popupTitle}>Stay Updated!</h3>
            <p className={styles.popupText}>
              Get the latest course updates, new modules, and exclusive robotics resources
              delivered to your inbox.
            </p>

            <form onSubmit={handleSubmit} className={styles.form}>
              <input
                type="email"
                placeholder="Enter your email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className={styles.emailInput}
                required
              />
              <button type="submit" className={styles.subscribeButton}>
                Subscribe Now
              </button>
            </form>

            <p className={styles.privacy}>
              🔒 We respect your privacy. Unsubscribe anytime.
            </p>
          </>
        ) : (
          <div className={styles.successMessage}>
            <div className={styles.successIcon}>✓</div>
            <h3 className={styles.successTitle}>Thank You!</h3>
            <p className={styles.successText}>You're all set! Check your inbox for updates.</p>
          </div>
        )}
      </div>
    </div>
  );
}
