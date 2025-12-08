import React, { useState } from 'react';
import styles from './ForgotPasswordModal.module.css';
import { authAPI } from '../services/api.service';

export default function ForgotPasswordModal({ isOpen, onClose }) {
  const [email, setEmail] = useState('');
  const [isSubmitted, setIsSubmitted] = useState(false);
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    if (!email) {
      setError('Email is required');
      return;
    }

    if (!/\S+@\S+\.\S+/.test(email)) {
      setError('Please enter a valid email');
      return;
    }

    setIsLoading(true);

    try {
      await authAPI.forgotPassword(email);
      setIsSubmitted(true);
    } catch (error) {
      setError(error.message || 'Failed to send reset email. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleClose = () => {
    setEmail('');
    setError('');
    setIsSubmitted(false);
    setIsLoading(false);
    onClose();
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={handleClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={handleClose}>×</button>

        {!isSubmitted ? (
          <>
            <div className={styles.modalHeader}>
              <div className={styles.iconWrapper}>
                <span className={styles.icon}>🔐</span>
              </div>
              <h2 className={styles.title}>Reset Your Password</h2>
              <p className={styles.subtitle}>
                Enter your email address and we'll send you instructions to reset your password.
              </p>
            </div>

            <form onSubmit={handleSubmit} className={styles.form}>
              <div className={styles.formGroup}>
                <label htmlFor="reset-email" className={styles.label}>
                  Email Address
                </label>
                <input
                  id="reset-email"
                  type="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  className={`${styles.input} ${error ? styles.inputError : ''}`}
                  placeholder="your@email.com"
                  autoFocus
                />
                {error && (
                  <span className={styles.errorText}>{error}</span>
                )}
              </div>

              <button
                type="submit"
                className={styles.submitButton}
                disabled={isLoading}>
                {isLoading ? (
                  <>
                    <span className={styles.spinner}></span>
                    Sending...
                  </>
                ) : (
                  'Send Reset Link'
                )}
              </button>
            </form>

            <div className={styles.helpText}>
              <p>Remember your password? <button onClick={handleClose} className={styles.linkButton}>Back to login</button></p>
            </div>
          </>
        ) : (
          <div className={styles.successState}>
            <div className={styles.successIcon}>✓</div>
            <h2 className={styles.successTitle}>Check Your Email</h2>
            <p className={styles.successText}>
              We've sent password reset instructions to <strong>{email}</strong>
            </p>
            <p className={styles.successSubtext}>
              Didn't receive the email? Check your spam folder or try again.
            </p>
            <button onClick={handleClose} className={styles.doneButton}>
              Done
            </button>
          </div>
        )}
      </div>
    </div>
  );
}
