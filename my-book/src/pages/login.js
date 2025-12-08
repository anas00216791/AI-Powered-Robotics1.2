import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import { useHistory, useLocation } from '@docusaurus/router';
import styles from './auth.module.css';
import ForgotPasswordModal from '../components/ForgotPasswordModal';
import { authAPI } from '../services/api.service';

export default function Login() {
  const history = useHistory();
  const location = useLocation();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [rememberMe, setRememberMe] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [errors, setErrors] = useState({});
  const [showForgotPassword, setShowForgotPassword] = useState(false);
  const [apiError, setApiError] = useState('');

  // Check for OAuth errors in URL
  useEffect(() => {
    const searchParams = new URLSearchParams(location.search);
    const error = searchParams.get('error');
    const provider = searchParams.get('provider');

    if (error === 'oauth_not_configured' && provider) {
      const providerName = provider.charAt(0).toUpperCase() + provider.slice(1);
      setApiError(
        `${providerName} OAuth is not configured yet. Please set up ${providerName} OAuth credentials in the backend .env file. See OAUTH_SETUP.md for instructions.`
      );
    } else if (error === 'authentication_failed') {
      setApiError('Authentication failed. Please try again.');
    } else if (error === 'oauth_failed') {
      setApiError('OAuth authentication failed. Please try again.');
    } else if (error === 'server_error') {
      setApiError('Server error occurred. Please try again later.');
    }
  }, [location]);

  const validateForm = () => {
    const newErrors = {};

    if (!email) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(email)) {
      newErrors.email = 'Email is invalid';
    }

    if (!password) {
      newErrors.password = 'Password is required';
    } else if (password.length < 6) {
      newErrors.password = 'Password must be at least 6 characters';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setApiError('');

    if (!validateForm()) return;

    setIsLoading(true);

    try {
      const response = await authAPI.login({
        email,
        password,
        rememberMe
      });

      if (response.success) {
        // Redirect to intro page or dashboard
        history.push('/intro');
      }
    } catch (error) {
      setApiError(error.message || 'Login failed. Please try again.');
      console.error('Login error:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSocialLogin = (provider) => {
    const API_BASE_URL = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL)
      ? process.env.REACT_APP_API_URL
      : 'http://localhost:5000/api';

    if (provider === 'Google') {
      window.location.href = `${API_BASE_URL}/auth/google`;
    } else if (provider === 'GitHub') {
      window.location.href = `${API_BASE_URL}/auth/github`;
    }
  };

  return (
    <Layout
      title="Login"
      description="Login to AI-Powered Robotics Course">
      <ForgotPasswordModal
        isOpen={showForgotPassword}
        onClose={() => setShowForgotPassword(false)}
      />
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <div className={styles.authIcon}>🤖</div>
            <h1 className={styles.authTitle}>Welcome Back!</h1>
            <p className={styles.authSubtitle}>Login to continue your learning journey</p>
          </div>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {apiError && (
              <div className={styles.alert} style={{
                padding: '1rem',
                marginBottom: '1.5rem',
                backgroundColor: '#fee',
                border: '1px solid #fcc',
                borderRadius: '8px',
                color: '#c33'
              }}>
                {apiError}
              </div>
            )}

            <div className={styles.formGroup}>
              <label htmlFor="email" className={styles.label}>
                Email Address
              </label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className={`${styles.input} ${errors.email ? styles.inputError : ''}`}
                placeholder="your@email.com"
              />
              {errors.email && (
                <span className={styles.errorText}>{errors.email}</span>
              )}
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password" className={styles.label}>
                Password
              </label>
              <div className={styles.passwordWrapper}>
                <input
                  id="password"
                  type={showPassword ? 'text' : 'password'}
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  className={`${styles.input} ${errors.password ? styles.inputError : ''}`}
                  placeholder="Enter your password"
                />
                <button
                  type="button"
                  onClick={() => setShowPassword(!showPassword)}
                  className={styles.passwordToggle}
                  aria-label="Toggle password visibility">
                  {showPassword ? '👁️' : '👁️‍🗨️'}
                </button>
              </div>
              {errors.password && (
                <span className={styles.errorText}>{errors.password}</span>
              )}
            </div>

            <div className={styles.formOptions}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={rememberMe}
                  onChange={(e) => setRememberMe(e.target.checked)}
                  className={styles.checkbox}
                />
                <span>Remember me</span>
              </label>
              <button
                type="button"
                onClick={() => setShowForgotPassword(true)}
                className={styles.forgotLink}>
                Forgot password?
              </button>
            </div>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}>
              {isLoading ? (
                <>
                  <span className={styles.spinner}></span>
                  Logging in...
                </>
              ) : (
                'Login'
              )}
            </button>
          </form>
          <div className={styles.authFooter}>
            <p>
              Don't have an account?{' '}
              <Link to="/signup" className={styles.authLink}>
                Sign up
              </Link>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
