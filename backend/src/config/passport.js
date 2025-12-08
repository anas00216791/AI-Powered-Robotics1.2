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

// Google OAuth Strategy (only if credentials are provided)
if (process.env.GOOGLE_CLIENT_ID &&
    process.env.GOOGLE_CLIENT_SECRET &&
    !process.env.GOOGLE_CLIENT_ID.includes('your-google-client-id')) {
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
} else {
  console.warn('⚠️  Google OAuth not configured. Set GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET in .env to enable.');
}

// GitHub OAuth Strategy (only if credentials are provided)
if (process.env.GITHUB_CLIENT_ID &&
    process.env.GITHUB_CLIENT_SECRET &&
    !process.env.GITHUB_CLIENT_ID.includes('your-github-client-id')) {
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
} else {
  console.warn('⚠️  GitHub OAuth not configured. Set GITHUB_CLIENT_ID and GITHUB_CLIENT_SECRET in .env to enable.');
}

module.exports = passport;
