import React, { useEffect, useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import Layout from '@theme/Layout';

export default function OAuthCallback() {
  const history = useHistory();
  const location = useLocation();
  const [status, setStatus] = useState('processing');
  const [message, setMessage] = useState('Authenticating...');

  useEffect(() => {
    const handleCallback = () => {
      const searchParams = new URLSearchParams(location.search);
      const token = searchParams.get('token');
      const refreshToken = searchParams.get('refreshToken');
      const error = searchParams.get('error');

      if (error) {
        setStatus('error');
        switch (error) {
          case 'authentication_failed':
            setMessage('Authentication failed. Please try again.');
            break;
          case 'oauth_failed':
            setMessage('OAuth authentication failed. Please try again.');
            break;
          case 'server_error':
            setMessage('Server error occurred. Please try again later.');
            break;
          default:
            setMessage('An error occurred during authentication.');
        }

        setTimeout(() => {
          history.push('/login');
        }, 3000);
        return;
      }

      if (token && refreshToken) {
        // Store tokens in localStorage
        localStorage.setItem('token', token);
        localStorage.setItem('refreshToken', refreshToken);

        setStatus('success');
        setMessage('Authentication successful! Redirecting...');

        // Redirect to dashboard/intro page
        setTimeout(() => {
          history.push('/intro');
        }, 1500);
      } else {
        setStatus('error');
        setMessage('Invalid authentication response. Please try again.');

        setTimeout(() => {
          history.push('/login');
        }, 3000);
      }
    };

    handleCallback();
  }, [location, history]);

  return (
    <Layout
      title="Authentication"
      description="OAuth authentication callback">
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '60vh',
        padding: '2rem'
      }}>
        <div style={{
          textAlign: 'center',
          maxWidth: '500px',
          padding: '3rem',
          backgroundColor: 'var(--ifm-background-surface-color)',
          borderRadius: '12px',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)'
        }}>
          {status === 'processing' && (
            <>
              <div style={{
                fontSize: '3rem',
                marginBottom: '1.5rem',
                animation: 'spin 1s linear infinite'
              }}>
                🔄
              </div>
              <h2 style={{ marginBottom: '1rem' }}>{message}</h2>
              <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                Please wait while we complete the authentication...
              </p>
            </>
          )}

          {status === 'success' && (
            <>
              <div style={{
                fontSize: '3rem',
                marginBottom: '1.5rem'
              }}>
                ✅
              </div>
              <h2 style={{ marginBottom: '1rem', color: 'var(--ifm-color-success)' }}>
                {message}
              </h2>
              <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                You'll be redirected to your dashboard shortly.
              </p>
            </>
          )}

          {status === 'error' && (
            <>
              <div style={{
                fontSize: '3rem',
                marginBottom: '1.5rem'
              }}>
                ❌
              </div>
              <h2 style={{ marginBottom: '1rem', color: 'var(--ifm-color-danger)' }}>
                {message}
              </h2>
              <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                Redirecting to login page...
              </p>
            </>
          )}
        </div>
      </div>

      <style>{`
        @keyframes spin {
          from {
            transform: rotate(0deg);
          }
          to {
            transform: rotate(360deg);
          }
        }
      `}</style>
    </Layout>
  );
}
