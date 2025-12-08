# OAuth Setup Guide

This guide will help you set up Google and GitHub OAuth authentication for the AI-Powered Robotics Course platform.

## Overview

The platform now supports three authentication methods:
1. **Traditional Email/Password** - Users can sign up with email and password
2. **Google OAuth** - Users can sign in with their Google account
3. **GitHub OAuth** - Users can sign in with their GitHub account

## Prerequisites

- Node.js and npm installed
- Backend server running on http://localhost:5000
- Frontend server running on http://localhost:3000

## Setup Instructions

### 1. Google OAuth Setup

#### Step 1: Create a Google Cloud Project

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Click on "APIs & Services" > "Credentials"

#### Step 2: Configure OAuth Consent Screen

1. Click "OAuth consent screen" in the left sidebar
2. Select "External" user type and click "Create"
3. Fill in the required fields:
   - App name: `AI-Powered Robotics Course`
   - User support email: Your email
   - Developer contact: Your email
4. Click "Save and Continue"
5. On the Scopes screen, click "Save and Continue"
6. On Test users, you can add test emails (optional for development)
7. Click "Save and Continue"

#### Step 3: Create OAuth Credentials

1. Go back to "Credentials"
2. Click "Create Credentials" > "OAuth client ID"
3. Select "Web application"
4. Configure:
   - Name: `Robotics Course Web Client`
   - Authorized JavaScript origins:
     - `http://localhost:3000`
     - `http://localhost:5000`
   - Authorized redirect URIs:
     - `http://localhost:5000/api/auth/google/callback`
5. Click "Create"
6. **Copy the Client ID and Client Secret** - you'll need these for your `.env` file

### 2. GitHub OAuth Setup

#### Step 1: Create a GitHub OAuth App

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Click "OAuth Apps" > "New OAuth App"
3. Fill in the application details:
   - Application name: `AI-Powered Robotics Course`
   - Homepage URL: `http://localhost:3000`
   - Application description: `Learning platform for AI-powered robotics`
   - Authorization callback URL: `http://localhost:5000/api/auth/github/callback`
4. Click "Register application"

#### Step 2: Generate Client Secret

1. After creating the app, you'll see your **Client ID**
2. Click "Generate a new client secret"
3. **Copy both the Client ID and Client Secret** - you'll need these for your `.env` file

**Important:** Make sure your GitHub email is public:
- Go to [GitHub Email Settings](https://github.com/settings/emails)
- Uncheck "Keep my email addresses private"
- Or ensure at least one email is marked as public

### 3. Configure Backend Environment Variables

Edit `backend/.env` file and add your OAuth credentials:

```env
# OAuth Configuration
# Google OAuth (from Google Cloud Console)
GOOGLE_CLIENT_ID=your-google-client-id-here.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret-here

# GitHub OAuth (from GitHub Developer Settings)
GITHUB_CLIENT_ID=your-github-client-id-here
GITHUB_CLIENT_SECRET=your-github-client-secret-here

# Session Secret (generate a random string)
SESSION_SECRET=change-this-to-a-random-secret-string

# Backend URL (for OAuth callbacks)
BACKEND_URL=http://localhost:5000

# Frontend URL (for redirects after OAuth)
FRONTEND_URL=http://localhost:3000
```

**Security Note:** Never commit real credentials to version control. Keep your `.env` file in `.gitignore`.

### 4. Restart Servers

After updating the environment variables:

1. Stop the backend server (Ctrl+C)
2. Restart it: `cd backend && npm start`
3. The frontend should automatically detect the changes

## Testing the OAuth Flow

### Test Google OAuth:

1. Open http://localhost:3000/login
2. Click "Continue with Google"
3. You'll be redirected to Google's login page
4. Sign in with your Google account
5. Grant permissions to the app
6. You'll be redirected back to your dashboard

### Test GitHub OAuth:

1. Open http://localhost:3000/login
2. Click "Continue with GitHub"
3. You'll be redirected to GitHub's authorization page
4. Authorize the application
5. You'll be redirected back to your dashboard

## How It Works

### OAuth Flow Diagram:

```
User clicks "Continue with Google/GitHub"
    ↓
Frontend redirects to: /api/auth/google or /api/auth/github
    ↓
Backend redirects to: OAuth provider (Google/GitHub)
    ↓
User authenticates with OAuth provider
    ↓
OAuth provider redirects to: /api/auth/google/callback or /api/auth/github/callback
    ↓
Backend processes OAuth response:
    - Fetches user profile
    - Creates or updates user in database
    - Generates JWT token and refresh token
    ↓
Backend redirects to: /auth/callback?token=xxx&refreshToken=yyy
    ↓
Frontend callback page:
    - Extracts tokens from URL
    - Stores tokens in localStorage
    - Redirects to dashboard
```

## Database Schema Changes

The User model now includes OAuth-specific fields:

```javascript
{
  oauthProvider: String,  // 'google', 'github', or null
  oauthId: String,        // Unique ID from OAuth provider
  avatar: String,         // Profile picture URL from OAuth provider
  // ... other fields
}
```

## Security Features

1. **Secure Sessions:** Express sessions with HTTP-only cookies
2. **JWT Tokens:** Access tokens and refresh tokens for API authentication
3. **CORS Protection:** Only allows requests from configured frontend URL
4. **Password Optional:** OAuth users don't need a password
5. **Email Verification:** OAuth users are automatically verified
6. **Provider Linking:** Users can link multiple OAuth providers to one account

## Troubleshooting

### Issue: "OAuth authentication failed"

**Solution:**
- Check that your OAuth credentials are correct in `.env`
- Verify redirect URIs match exactly in OAuth provider settings
- Ensure servers are running on correct ports

### Issue: "No email found in profile"

**For GitHub:**
- Make sure your email is public in GitHub settings
- Go to https://github.com/settings/emails
- Uncheck "Keep my email addresses private"

**For Google:**
- Email scope is included by default
- Check OAuth consent screen configuration

### Issue: "Redirect URI mismatch"

**Solution:**
- Ensure callback URLs in OAuth provider settings exactly match:
  - Google: `http://localhost:5000/api/auth/google/callback`
  - GitHub: `http://localhost:5000/api/auth/github/callback`
- No trailing slashes
- Use http (not https) for local development

### Issue: Backend crashes on OAuth callback

**Solution:**
- Check backend logs for specific error
- Verify all required packages are installed: `npm install`
- Ensure MongoDB connection or in-memory mode is working

## Production Deployment

When deploying to production:

1. **Update URLs in OAuth Provider Settings:**
   - Replace `http://localhost:3000` with your production frontend URL
   - Replace `http://localhost:5000` with your production backend URL

2. **Update Environment Variables:**
   ```env
   FRONTEND_URL=https://your-domain.com
   BACKEND_URL=https://api.your-domain.com
   NODE_ENV=production
   ```

3. **Enable HTTPS:**
   - OAuth providers require HTTPS for production
   - Update session cookie settings:
     ```javascript
     cookie: {
       secure: true,  // Requires HTTPS
       httpOnly: true,
       sameSite: 'lax'
     }
     ```

4. **Generate Strong Secrets:**
   - Use a cryptographically secure random string for `SESSION_SECRET`
   - Never use the same secrets as development

5. **Enable OAuth Consent Screen:**
   - For Google: Change from "Testing" to "In production"
   - For GitHub: No special steps needed

## File Structure

```
backend/
├── src/
│   ├── config/
│   │   └── passport.js           # Passport OAuth strategies
│   ├── controllers/
│   │   ├── auth.controller.js    # Traditional auth
│   │   └── oauth.controller.js   # OAuth callbacks
│   ├── models/
│   │   └── User.model.js         # User schema with OAuth fields
│   ├── routes/
│   │   ├── auth.routes.js        # Traditional auth routes
│   │   └── oauth.routes.js       # OAuth routes
│   └── server.js                 # Express app with Passport

my-book/
├── src/
│   └── pages/
│       ├── login.js              # Login page with OAuth buttons
│       └── auth/
│           └── callback.js       # OAuth callback handler
```

## API Endpoints

### OAuth Endpoints:

- `GET /api/auth/google` - Initiates Google OAuth flow
- `GET /api/auth/google/callback` - Google OAuth callback
- `GET /api/auth/github` - Initiates GitHub OAuth flow
- `GET /api/auth/github/callback` - GitHub OAuth callback
- `GET /api/auth/oauth/failure` - OAuth failure handler

### Traditional Auth Endpoints:

- `POST /api/auth/signup` - Email/password signup
- `POST /api/auth/login` - Email/password login
- `GET /api/auth/me` - Get current user (protected)
- `POST /api/auth/logout` - Logout (protected)
- `POST /api/auth/forgot-password` - Request password reset
- `POST /api/auth/reset-password/:token` - Reset password

## Support

If you encounter any issues:

1. Check the backend console for error messages
2. Check the browser console for frontend errors
3. Verify all environment variables are set correctly
4. Ensure OAuth redirect URIs match exactly
5. Try clearing browser cache and localStorage

## Next Steps

After setting up OAuth:

1. Test both Google and GitHub authentication
2. Verify user creation in database
3. Test linking multiple OAuth providers to one account
4. Configure production OAuth apps
5. Add additional OAuth providers (Facebook, Twitter, etc.) if needed

---

**Happy coding! 🚀**
