const express = require('express');
const router = express.Router();
const passport = require('../config/passport');
const oauthController = require('../controllers/oauth.controller');

// Middleware to check if OAuth provider is configured
const checkOAuthProvider = (provider) => {
  return (req, res, next) => {
    const strategy = passport._strategy(provider);

    if (!strategy) {
      return res.redirect(
        `${process.env.FRONTEND_URL || 'http://localhost:3000'}/login?error=oauth_not_configured&provider=${provider}`
      );
    }

    next();
  };
};

// Google OAuth routes
router.get('/google',
  checkOAuthProvider('google'),
  passport.authenticate('google', { scope: ['profile', 'email'] })
);

router.get('/google/callback',
  checkOAuthProvider('google'),
  passport.authenticate('google', {
    failureRedirect: '/api/auth/oauth/failure',
    session: false
  }),
  oauthController.oauthSuccess
);

// GitHub OAuth routes
router.get('/github',
  checkOAuthProvider('github'),
  passport.authenticate('github', { scope: ['user:email'] })
);

router.get('/github/callback',
  checkOAuthProvider('github'),
  passport.authenticate('github', {
    failureRedirect: '/api/auth/oauth/failure',
    session: false
  }),
  oauthController.oauthSuccess
);

// OAuth failure route
router.get('/oauth/failure', oauthController.oauthFailure);

module.exports = router;
