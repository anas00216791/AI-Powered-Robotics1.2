# OAuth Authentication - Complete Working Code

This document contains all the complete working code for Google and GitHub OAuth authentication.

## Table of Contents
1. [Backend Code](#backend-code)
2. [Frontend Code](#frontend-code)
3. [Configuration](#configuration)
4. [Quick Start](#quick-start)

---

## Backend Code

### 1. User Model (`backend/src/models/User.model.js`)

```javascript
const mongoose = require('mongoose');
const bcrypt = require('bcryptjs');

const userSchema = new mongoose.Schema({
  fullName: {
    type: String,
    required: [true, 'Full name is required'],
    trim: true,
    minlength: [2, 'Name must be at least 2 characters'],
    maxlength: [50, 'Name cannot exceed 50 characters']
  },
  email: {
    type: String,
    required: [true, 'Email is required'],
    unique: true,
    lowercase: true,
    trim: true,
    match: [
      /^\w+([\.-]?\w+)*@\w+([\.-]?\w+)*(\.\w{2,3})+$/,
      'Please provide a valid email'
    ]
  },
  password: {
    type: String,
    required: function() {
      // Password only required if not using OAuth
      return !this.oauthProvider;
    },
    minlength: [8, 'Password must be at least 8 characters'],
    select: false // Don't return password by default
  },
  oauthProvider: {
    type: String,
    enum: ['google', 'github', null],
    default: null
  },
  oauthId: {
    type: String,
    sparse: true,
    unique: true
  },
  avatar: {
    type: String
  },
  role: {
    type: String,
    enum: ['student', 'instructor', 'admin'],
    default: 'student'
  },
  isVerified: {
    type: Boolean,
    default: false
  },
  progress: {
    type: Map,
    of: Number,
    default: {}
  },
  enrolledModules: [{
    type: String
  }],
  resetPasswordToken: String,
  resetPasswordExpire: Date,
  lastLogin: Date,
  createdAt: {
    type: Date,
    default: Date.now
  },
  updatedAt: {
    type: Date,
    default: Date.now
  }
}, {
  timestamps: true
});

// Hash password before saving
userSchema.pre('save', async function(next) {
  // Skip password hashing for OAuth users
  if (!this.isModified('password') || !this.password) {
    return next();
  }

  try {
    const salt = await bcrypt.genSalt(10);
    this.password = await bcrypt.hash(this.password, salt);
    next();
  } catch (error) {
    next(error);
  }
});

// Compare password method
userSchema.methods.comparePassword = async function(candidatePassword) {
  try {
    return await bcrypt.compare(candidatePassword, this.password);
  } catch (error) {
    throw new Error('Password comparison failed');
  }
};

// Generate reset password token
userSchema.methods.generateResetToken = function() {
  const resetToken = Math.random().toString(36).substring(2, 15) +
                     Math.random().toString(36).substring(2, 15);

  this.resetPasswordToken = require('crypto')
    .createHash('sha256')
    .update(resetToken)
    .digest('hex');

  this.resetPasswordExpire = Date.now() + parseInt(process.env.RESET_PASSWORD_EXPIRE || 3600000);

  return resetToken;
};

// Remove sensitive data from JSON output
userSchema.methods.toJSON = function() {
  const user = this.toObject();
  delete user.password;
  delete user.resetPasswordToken;
  delete user.resetPasswordExpire;
  return user;
};

module.exports = mongoose.model('User', userSchema);
```

### 2. Passport Configuration (`backend/src/config/passport.js`)

```javascript
const passport = require('passport');
const GoogleStrategy = require('passport-google-oauth20').Strategy;
const GitHubStrategy = require('passport-github2').Strategy;
const User = require('../models/User.model');
const mongoose = require('mongoose');
const memoryStore = require('../utils/memoryStore');

// Serialize user for session
passport.serializeUser((user, done) => {
  done(null, user.id || user._id);
});

// Deserialize user from session
passport.deserializeUser(async (id, done) => {
  try {
    const useMemoryStore = mongoose.connection.readyState !== 1;

    if (useMemoryStore) {
      const user = await memoryStore.findById(id);
      done(null, user);
    } else {
      const user = await User.findById(id);
      done(null, user);
    }
  } catch (error) {
    done(error, null);
  }
});

// Google OAuth Strategy
passport.use(new GoogleStrategy({
  clientID: process.env.GOOGLE_CLIENT_ID,
  clientSecret: process.env.GOOGLE_CLIENT_SECRET,
  callbackURL: `${process.env.BACKEND_URL || 'http://localhost:5000'}/api/auth/google/callback`,
  scope: ['profile', 'email']
},
async (accessToken, refreshToken, profile, done) => {
  try {
    const useMemoryStore = mongoose.connection.readyState !== 1;

    const email = profile.emails && profile.emails[0] ? profile.emails[0].value : null;

    if (!email) {
      return done(new Error('No email found in Google profile'), null);
    }

    if (useMemoryStore) {
      // In-memory storage
      let user = await memoryStore.findByEmail(email);

      if (user) {
        // Update existing user
        user.oauthProvider = 'google';
        user.oauthId = profile.id;
        user.avatar = profile.photos && profile.photos[0] ? profile.photos[0].value : user.avatar;
        user.isVerified = true;
        user.lastLogin = new Date();
      } else {
        // Create new user
        user = await memoryStore.createUser({
          fullName: profile.displayName || 'Google User',
          email: email,
          oauthProvider: 'google',
          oauthId: profile.id,
          avatar: profile.photos && profile.photos[0] ? profile.photos[0].value : null,
          isVerified: true,
          lastLogin: new Date()
        });
      }

      return done(null, user);
    }

    // MongoDB storage
    let user = await User.findOne({
      $or: [
        { oauthId: profile.id, oauthProvider: 'google' },
        { email: email }
      ]
    });

    if (user) {
      // Update existing user with OAuth info
      user.oauthProvider = 'google';
      user.oauthId = profile.id;
      user.avatar = profile.photos && profile.photos[0] ? profile.photos[0].value : user.avatar;
      user.isVerified = true;
      user.lastLogin = new Date();
      await user.save();
    } else {
      // Create new user
      user = await User.create({
        fullName: profile.displayName || 'Google User',
        email: email,
        oauthProvider: 'google',
        oauthId: profile.id,
        avatar: profile.photos && profile.photos[0] ? profile.photos[0].value : null,
        isVerified: true,
        lastLogin: new Date()
      });
    }

    return done(null, user);
  } catch (error) {
    console.error('Google OAuth error:', error);
    return done(error, null);
  }
}));

// GitHub OAuth Strategy
passport.use(new GitHubStrategy({
  clientID: process.env.GITHUB_CLIENT_ID,
  clientSecret: process.env.GITHUB_CLIENT_SECRET,
  callbackURL: `${process.env.BACKEND_URL || 'http://localhost:5000'}/api/auth/github/callback`,
  scope: ['user:email']
},
async (accessToken, refreshToken, profile, done) => {
  try {
    const useMemoryStore = mongoose.connection.readyState !== 1;

    const email = profile.emails && profile.emails[0] ? profile.emails[0].value : null;

    if (!email) {
      return done(new Error('No email found in GitHub profile. Please make your email public in GitHub settings.'), null);
    }

    if (useMemoryStore) {
      // In-memory storage
      let user = await memoryStore.findByEmail(email);

      if (user) {
        // Update existing user
        user.oauthProvider = 'github';
        user.oauthId = profile.id;
        user.avatar = profile.photos && profile.photos[0] ? profile.photos[0].value : user.avatar;
        user.isVerified = true;
        user.lastLogin = new Date();
      } else {
        // Create new user
        user = await memoryStore.createUser({
          fullName: profile.displayName || profile.username || 'GitHub User',
          email: email,
          oauthProvider: 'github',
          oauthId: profile.id,
          avatar: profile.photos && profile.photos[0] ? profile.photos[0].value : null,
          isVerified: true,
          lastLogin: new Date()
        });
      }

      return done(null, user);
    }

    // MongoDB storage
    let user = await User.findOne({
      $or: [
        { oauthId: profile.id, oauthProvider: 'github' },
        { email: email }
      ]
    });

    if (user) {
      // Update existing user with OAuth info
      user.oauthProvider = 'github';
      user.oauthId = profile.id;
      user.avatar = profile.photos && profile.photos[0] ? profile.photos[0].value : user.avatar;
      user.isVerified = true;
      user.lastLogin = new Date();
      await user.save();
    } else {
      // Create new user
      user = await User.create({
        fullName: profile.displayName || profile.username || 'GitHub User',
        email: email,
        oauthProvider: 'github',
        oauthId: profile.id,
        avatar: profile.photos && profile.photos[0] ? profile.photos[0].value : null,
        isVerified: true,
        lastLogin: new Date()
      });
    }

    return done(null, user);
  } catch (error) {
    console.error('GitHub OAuth error:', error);
    return done(error, null);
  }
}));

module.exports = passport;
```

### 3. OAuth Controller (`backend/src/controllers/oauth.controller.js`)

```javascript
const jwt = require('jsonwebtoken');

// Generate JWT Token
const generateToken = (userId) => {
  return jwt.sign({ id: userId }, process.env.JWT_SECRET, {
    expiresIn: process.env.JWT_EXPIRE || '7d'
  });
};

// Generate Refresh Token
const generateRefreshToken = (userId) => {
  return jwt.sign({ id: userId }, process.env.JWT_REFRESH_SECRET, {
    expiresIn: process.env.JWT_REFRESH_EXPIRE || '30d'
  });
};

// @desc    Handle OAuth success
// @route   GET /api/auth/google/callback
// @route   GET /api/auth/github/callback
// @access  Public
exports.oauthSuccess = async (req, res) => {
  try {
    if (!req.user) {
      return res.redirect(`${process.env.FRONTEND_URL || 'http://localhost:3000'}/login?error=authentication_failed`);
    }

    const userId = req.user._id || req.user.id;

    // Generate tokens
    const token = generateToken(userId);
    const refreshToken = generateRefreshToken(userId);

    // Redirect to frontend with tokens
    const redirectUrl = `${process.env.FRONTEND_URL || 'http://localhost:3000'}/auth/callback?token=${token}&refreshToken=${refreshToken}`;
    res.redirect(redirectUrl);
  } catch (error) {
    console.error('OAuth success error:', error);
    res.redirect(`${process.env.FRONTEND_URL || 'http://localhost:3000'}/login?error=server_error`);
  }
};

// @desc    Handle OAuth failure
// @access  Public
exports.oauthFailure = (req, res) => {
  console.error('OAuth authentication failed');
  res.redirect(`${process.env.FRONTEND_URL || 'http://localhost:3000'}/login?error=oauth_failed`);
};
```

### 4. OAuth Routes (`backend/src/routes/oauth.routes.js`)

```javascript
const express = require('express');
const router = express.Router();
const passport = require('../config/passport');
const oauthController = require('../controllers/oauth.controller');

// Google OAuth routes
router.get('/google',
  passport.authenticate('google', { scope: ['profile', 'email'] })
);

router.get('/google/callback',
  passport.authenticate('google', {
    failureRedirect: '/api/auth/oauth/failure',
    session: false
  }),
  oauthController.oauthSuccess
);

// GitHub OAuth routes
router.get('/github',
  passport.authenticate('github', { scope: ['user:email'] })
);

router.get('/github/callback',
  passport.authenticate('github', {
    failureRedirect: '/api/auth/oauth/failure',
    session: false
  }),
  oauthController.oauthSuccess
);

// OAuth failure route
router.get('/oauth/failure', oauthController.oauthFailure);

module.exports = router;
```

### 5. Updated Server (`backend/src/server.js`)

```javascript
const express = require('express');
const mongoose = require('mongoose');
const cors = require('cors');
const dotenv = require('dotenv');
const session = require('express-session');
const passport = require('./config/passport');

// Load environment variables
dotenv.config();

// Import routes
const authRoutes = require('./routes/auth.routes');
const oauthRoutes = require('./routes/oauth.routes');

// Initialize express app
const app = express();

// Middleware
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true
}));
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Session middleware for Passport
app.use(session({
  secret: process.env.SESSION_SECRET || 'your-secret-key-change-this',
  resave: false,
  saveUninitialized: false,
  cookie: {
    secure: process.env.NODE_ENV === 'production',
    httpOnly: true,
    maxAge: 24 * 60 * 60 * 1000 // 24 hours
  }
}));

// Initialize Passport
app.use(passport.initialize());
app.use(passport.session());

// Routes
app.use('/api/auth', authRoutes);
app.use('/api/auth', oauthRoutes);

// Health check endpoint
app.get('/api/health', (req, res) => {
  res.status(200).json({
    success: true,
    message: 'Server is running',
    timestamp: new Date().toISOString()
  });
});

// Error handling middleware
app.use((err, req, res, next) => {
  console.error(err.stack);
  res.status(err.statusCode || 500).json({
    success: false,
    message: err.message || 'Internal Server Error',
    errors: err.errors || []
  });
});

// 404 handler
app.use((req, res) => {
  res.status(404).json({
    success: false,
    message: 'Route not found'
  });
});

// Database connection
const connectDB = async () => {
  try {
    await mongoose.connect(process.env.MONGODB_URI, {
      useNewUrlParser: true,
      useUnifiedTopology: true,
      serverSelectionTimeoutMS: 5000,
    });
    console.log('✅ MongoDB connected successfully');
    return true;
  } catch (error) {
    console.warn('⚠️  MongoDB connection failed:', error.message);
    console.log('📝 Running in IN-MEMORY mode (data will not persist)');
    return false;
  }
};

// Start server
const PORT = process.env.PORT || 5000;

connectDB().then((isConnected) => {
  if (!isConnected) {
    console.log('💡 Tip: Install MongoDB or use MongoDB Atlas for data persistence');
  }

  app.listen(PORT, () => {
    console.log(`🚀 Server is running on port ${PORT}`);
    console.log(`📍 API URL: http://localhost:${PORT}/api`);
    console.log(`🏥 Health check: http://localhost:${PORT}/api/health`);
  });
});

// Handle unhandled promise rejections
process.on('unhandledRejection', (err) => {
  console.error('❌ Unhandled Rejection:', err.message);
  process.exit(1);
});
```

---

## Frontend Code

### 1. Login Page (`my-book/src/pages/login.js`)

**Key change - Update the `handleSocialLogin` function:**

```javascript
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
```

### 2. OAuth Callback Page (`my-book/src/pages/auth/callback.js`)

```javascript
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
```

---

## Configuration

### Backend Environment Variables (`backend/.env`)

```env
# Server Configuration
PORT=5000
NODE_ENV=development

# MongoDB Configuration
MONGODB_URI=mongodb://localhost:27017/robotics-course

# JWT Configuration
JWT_SECRET=your-super-secret-jwt-key-change-this-in-production
JWT_EXPIRE=7d
JWT_REFRESH_SECRET=your-super-secret-refresh-token-key
JWT_REFRESH_EXPIRE=30d

# Frontend URL (for CORS)
FRONTEND_URL=http://localhost:3000

# Backend URL (for OAuth callbacks)
BACKEND_URL=http://localhost:5000

# Session Secret
SESSION_SECRET=your-session-secret-change-this-in-production

# OAuth Configuration
# Google OAuth (Get from: https://console.cloud.google.com/apis/credentials)
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret

# GitHub OAuth (Get from: https://github.com/settings/developers)
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

---

## Quick Start

### 1. Install Dependencies

```bash
# Backend
cd backend
npm install passport passport-google-oauth20 passport-github2 express-session

# Frontend (if needed)
cd ../my-book
npm install
```

### 2. Configure OAuth Providers

#### Google OAuth:
1. Go to https://console.cloud.google.com/apis/credentials
2. Create OAuth 2.0 Client ID
3. Add callback URL: `http://localhost:5000/api/auth/google/callback`
4. Copy Client ID and Secret to `.env`

#### GitHub OAuth:
1. Go to https://github.com/settings/developers
2. Create new OAuth App
3. Add callback URL: `http://localhost:5000/api/auth/github/callback`
4. Copy Client ID and Secret to `.env`

### 3. Update Environment Variables

Edit `backend/.env` and add your OAuth credentials.

### 4. Start Servers

```bash
# Terminal 1 - Backend
cd backend
npm start

# Terminal 2 - Frontend
cd my-book
npm start
```

### 5. Test OAuth Flow

1. Open http://localhost:3000/login
2. Click "Continue with Google" or "Continue with GitHub"
3. Authenticate with the provider
4. Get redirected back to your dashboard

---

## API Endpoints

### OAuth Endpoints:
- `GET /api/auth/google` - Initiates Google OAuth flow
- `GET /api/auth/google/callback` - Google OAuth callback
- `GET /api/auth/github` - Initiates GitHub OAuth flow
- `GET /api/auth/github/callback` - GitHub OAuth callback

### Traditional Auth:
- `POST /api/auth/signup` - Register with email/password
- `POST /api/auth/login` - Login with email/password
- `GET /api/auth/me` - Get current user (requires JWT token)
- `POST /api/auth/logout` - Logout

---

## Security Best Practices

1. **Never commit `.env` file to version control**
2. **Use strong secrets** for JWT and session
3. **Enable HTTPS in production**
4. **Validate redirect URIs** match exactly
5. **Rate limit authentication endpoints**
6. **Monitor for suspicious login activity**

---

## Need Help?

See `OAUTH_SETUP.md` for detailed setup instructions and troubleshooting guide.

**Happy coding! 🚀**
