# Vercel Deployment Guide

This guide explains how to deploy your monorepo (Docusaurus frontend + Express backend) to Vercel.

## Project Structure

```
.
├── api/
│   └── index.js                 # Vercel serverless function entry point
├── backend/
│   ├── src/
│   │   ├── app.js              # Express app (exported for serverless)
│   │   └── server.js           # Local development server
│   └── package.json
├── my-book/                     # Docusaurus site
│   ├── docs/
│   ├── src/
│   ├── build/                   # Build output (generated)
│   └── package.json
├── vercel.json                  # Vercel configuration
└── .vercelignore               # Files to exclude from deployment
```

## Configuration Files

### 1. vercel.json
- Specifies build commands for Docusaurus
- Sets output directory to `my-book/build`
- Configures API routes to use serverless functions
- Rewrites `/api/*` requests to the serverless backend

### 2. api/index.js
- Entry point for Vercel serverless functions
- Exports the Express app without calling `.listen()`

### 3. backend/src/app.js
- Express app configuration
- Exports the app for use in serverless environment
- Handles database connections with singleton pattern

## Deployment Steps

### Method 1: Deploy via Vercel CLI

1. **Install Vercel CLI**
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**
   ```bash
   vercel login
   ```

3. **Deploy to production**
   ```bash
   vercel --prod
   ```

### Method 2: Deploy via Vercel Dashboard

1. **Go to [Vercel Dashboard](https://vercel.com/dashboard)**

2. **Import your Git repository**
   - Click "Add New Project"
   - Import your repository from GitHub/GitLab/Bitbucket

3. **Configure project settings** (if not auto-detected)
   - Framework Preset: **Docusaurus**
   - Root Directory: **Leave empty** (uses root)
   - Build Command: Already set in `vercel.json`
   - Output Directory: Already set in `vercel.json`

4. **Set environment variables** (see below)

5. **Deploy**
   - Click "Deploy"
   - Wait for the build to complete

## Environment Variables

Add these environment variables in the Vercel Dashboard (Settings → Environment Variables):

### Required for Backend

```bash
# MongoDB connection string
MONGODB_URI=mongodb+srv://username:password@cluster.mongodb.net/database?retryWrites=true&w=majority

# Session secret (generate a random string)
SESSION_SECRET=your-secret-key-here-min-32-chars

# Frontend URL (your Vercel deployment URL)
FRONTEND_URL=https://your-app.vercel.app

# Node environment
NODE_ENV=production
```

### Required for OAuth (if using)

```bash
# Google OAuth
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GOOGLE_CALLBACK_URL=https://your-app.vercel.app/api/auth/google/callback

# GitHub OAuth
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
GITHUB_CALLBACK_URL=https://your-app.vercel.app/api/auth/github/callback

# JWT Secret
JWT_SECRET=your-jwt-secret-key
```

## How It Works

1. **Frontend (Docusaurus)**
   - Builds static files to `my-book/build/`
   - Served as static assets by Vercel's edge network
   - Accessible at root path: `https://your-app.vercel.app/`

2. **Backend (Express API)**
   - Converted to serverless function in `api/index.js`
   - Runs on-demand when API routes are accessed
   - Accessible at: `https://your-app.vercel.app/api/*`

3. **Routing**
   - `/api/*` → Backend serverless function
   - `/*` → Docusaurus static site

## Testing Locally

Before deploying, test locally:

```bash
# Terminal 1: Start backend
cd backend
npm install
npm start

# Terminal 2: Start frontend
cd my-book
npm install
npm start
```

## Verifying Deployment

After deployment, test these endpoints:

1. **Frontend**: `https://your-app.vercel.app/`
2. **API Health**: `https://your-app.vercel.app/api/health`
3. **Auth endpoints**: `https://your-app.vercel.app/api/auth/*`

## Common Issues & Solutions

### 404 Error on API Routes

**Problem**: API routes return 404
**Solution**:
- Check `vercel.json` has correct rewrites
- Verify `api/index.js` exports the Express app
- Check build logs in Vercel dashboard

### Build Failures

**Problem**: Build fails during deployment
**Solution**:
- Check `my-book/package.json` has all dependencies
- Verify build command works locally: `cd my-book && npm run build`
- Review build logs in Vercel dashboard

### MongoDB Connection Errors

**Problem**: Database connection fails
**Solution**:
- Verify `MONGODB_URI` environment variable is set
- Check MongoDB Atlas whitelist includes `0.0.0.0/0` (allow all) for Vercel
- Ensure MongoDB user has correct permissions

### Session/OAuth Issues

**Problem**: Authentication not working
**Solution**:
- Set `SESSION_SECRET` environment variable
- Update OAuth callback URLs in provider dashboards (Google, GitHub)
- Set `FRONTEND_URL` to your Vercel deployment URL

### Cold Start Issues

**Problem**: First API request is slow
**Solution**:
- This is normal for serverless functions (cold start)
- Consider upgrading to Vercel Pro for better performance
- Use MongoDB connection pooling (already implemented)

## Advanced Configuration

### Custom Domain

1. Go to Vercel Dashboard → Settings → Domains
2. Add your custom domain
3. Update DNS records as instructed
4. Update environment variables with new domain

### Incremental Static Regeneration (ISR)

If you want to enable ISR for Docusaurus:
- Add `trailingSlash: false` to `docusaurus.config.js`
- Configure ISR in `vercel.json`

### Preview Deployments

Every git push creates a preview deployment:
- Preview URL: `https://your-app-git-branch.vercel.app`
- Environment variables from Vercel dashboard are included

## Resources

- [Vercel Documentation](https://vercel.com/docs)
- [Docusaurus Deployment](https://docusaurus.io/docs/deployment)
- [Vercel Serverless Functions](https://vercel.com/docs/functions)
- [MongoDB Atlas](https://www.mongodb.com/cloud/atlas)

## Troubleshooting

If you encounter issues:

1. Check Vercel build logs
2. Review function logs in Vercel dashboard
3. Test API endpoints with tools like Postman
4. Verify all environment variables are set correctly
5. Check MongoDB connection from your IP address

## Support

For issues specific to:
- Vercel: [Vercel Support](https://vercel.com/support)
- Docusaurus: [Docusaurus Community](https://discord.gg/docusaurus)
- MongoDB: [MongoDB Community](https://www.mongodb.com/community/forums/)
