# AI-Powered Robotics Course - Backend API

Complete authentication backend for the AI-Powered Robotics Course platform.

## Features

- ✅ User Registration (Signup)
- ✅ User Login with JWT
- ✅ Password Reset (Forgot Password)
- ✅ Email Notifications
- ✅ Protected Routes with JWT Authentication
- ✅ Input Validation
- ✅ MongoDB Integration
- ✅ Secure Password Hashing with bcrypt
- ✅ CORS Enabled for Frontend

## Tech Stack

- **Node.js** & **Express.js** - Server framework
- **MongoDB** & **Mongoose** - Database
- **JWT** - Authentication tokens
- **bcryptjs** - Password hashing
- **Nodemailer** - Email sending
- **express-validator** - Input validation

## Project Structure

```
backend/
├── src/
│   ├── controllers/         # Route controllers
│   │   └── auth.controller.js
│   ├── models/             # Database models
│   │   └── User.model.js
│   ├── routes/             # API routes
│   │   └── auth.routes.js
│   ├── middleware/         # Custom middleware
│   │   ├── auth.middleware.js
│   │   └── validate.middleware.js
│   ├── utils/              # Utility functions
│   │   └── email.js
│   └── server.js           # Entry point
├── .env.example            # Environment variables template
├── package.json
└── README.md
```

## Installation

### 1. Install Dependencies

```bash
cd backend
npm install
```

### 2. Set Up Environment Variables

Create a `.env` file in the backend directory:

```bash
cp .env.example .env
```

Update `.env` with your configuration:

```env
# Server
PORT=5000
NODE_ENV=development

# MongoDB
MONGODB_URI=mongodb://localhost:27017/robotics-course

# JWT
JWT_SECRET=your-super-secret-jwt-key-change-this-in-production
JWT_EXPIRE=7d
JWT_REFRESH_SECRET=your-refresh-token-secret
JWT_REFRESH_EXPIRE=30d

# Email (Gmail example)
EMAIL_SERVICE=gmail
EMAIL_HOST=smtp.gmail.com
EMAIL_PORT=587
EMAIL_USER=your-email@gmail.com
EMAIL_PASSWORD=your-app-specific-password
EMAIL_FROM=noreply@roboticscourse.com

# Frontend
FRONTEND_URL=http://localhost:3000

# Password Reset
RESET_PASSWORD_EXPIRE=3600000
```

### 3. Set Up MongoDB

**Option A: Local MongoDB**
```bash
# Install MongoDB on your system
# Windows: https://www.mongodb.com/try/download/community
# Mac: brew install mongodb-community
# Linux: sudo apt-get install mongodb

# Start MongoDB
mongod
```

**Option B: MongoDB Atlas (Cloud)**
1. Create account at https://www.mongodb.com/atlas
2. Create a cluster
3. Get connection string
4. Update `MONGODB_URI` in `.env`

### 4. Set Up Email (Gmail Example)

1. Enable 2-Factor Authentication on your Gmail account
2. Generate an App Password:
   - Go to Google Account Settings
   - Security → 2-Step Verification → App Passwords
   - Generate password for "Mail"
3. Use the app password in `EMAIL_PASSWORD`

## Running the Server

### Development Mode (with auto-reload)

```bash
npm run dev
```

### Production Mode

```bash
npm start
```

Server will run on `http://localhost:5000`

## API Endpoints

### Public Routes

#### 1. Register User
```http
POST /api/auth/signup
Content-Type: application/json

{
  "fullName": "John Doe",
  "email": "john@example.com",
  "password": "SecurePass123"
}
```

**Response:**
```json
{
  "success": true,
  "message": "User registered successfully",
  "data": {
    "user": {
      "id": "...",
      "fullName": "John Doe",
      "email": "john@example.com",
      "role": "student"
    },
    "token": "jwt-token",
    "refreshToken": "refresh-token"
  }
}
```

#### 2. Login
```http
POST /api/auth/login
Content-Type: application/json

{
  "email": "john@example.com",
  "password": "SecurePass123",
  "rememberMe": false
}
```

**Response:**
```json
{
  "success": true,
  "message": "Login successful",
  "data": {
    "user": { ... },
    "token": "jwt-token",
    "refreshToken": "refresh-token"
  }
}
```

#### 3. Forgot Password
```http
POST /api/auth/forgot-password
Content-Type: application/json

{
  "email": "john@example.com"
}
```

**Response:**
```json
{
  "success": true,
  "message": "Password reset email sent"
}
```

#### 4. Reset Password
```http
POST /api/auth/reset-password/:token
Content-Type: application/json

{
  "password": "NewSecurePass123"
}
```

### Protected Routes (Require JWT Token)

#### 5. Get Current User
```http
GET /api/auth/me
Authorization: Bearer <jwt-token>
```

**Response:**
```json
{
  "success": true,
  "data": {
    "user": { ... }
  }
}
```

#### 6. Logout
```http
POST /api/auth/logout
Authorization: Bearer <jwt-token>
```

## Testing the API

### Using cURL

**Signup:**
```bash
curl -X POST http://localhost:5000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"fullName":"John Doe","email":"john@example.com","password":"SecurePass123"}'
```

**Login:**
```bash
curl -X POST http://localhost:5000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"john@example.com","password":"SecurePass123"}'
```

### Using Postman

1. Import the collection from the API endpoints above
2. Set environment variable `BASE_URL` = `http://localhost:5000/api`
3. After login, save the token
4. For protected routes, add header: `Authorization: Bearer <token>`

## Frontend Integration

The frontend is already configured to use this backend!

**Update frontend environment:**

Create `my-book/.env` file:
```env
REACT_APP_API_URL=http://localhost:5000/api
```

The frontend will automatically:
- Call the backend API for login/signup
- Store JWT tokens in localStorage
- Include tokens in protected requests
- Redirect after successful authentication

## Database Schema

### User Model

```javascript
{
  fullName: String (required, 2-50 chars),
  email: String (required, unique, valid email),
  password: String (required, min 8 chars, hashed),
  role: String (enum: student, instructor, admin),
  isVerified: Boolean (default: false),
  progress: Map,
  enrolledModules: [String],
  resetPasswordToken: String,
  resetPasswordExpire: Date,
  lastLogin: Date,
  createdAt: Date,
  updatedAt: Date
}
```

## Security Features

- ✅ Passwords hashed with bcrypt (10 rounds)
- ✅ JWT tokens with expiration
- ✅ Input validation and sanitization
- ✅ CORS protection
- ✅ Password reset tokens (SHA-256 hashed)
- ✅ Rate limiting ready (can be added)
- ✅ HTTP-only cookies support (can be enabled)

## Error Handling

All errors return consistent format:

```json
{
  "success": false,
  "message": "Error description",
  "errors": ["Validation error 1", "Validation error 2"]
}
```

## Troubleshooting

### MongoDB Connection Error
- Ensure MongoDB is running
- Check connection string in `.env`
- For Atlas, whitelist your IP address

### Email Not Sending
- Verify Gmail credentials
- Use app-specific password (not regular password)
- Check firewall/antivirus settings
- Try different email service

### CORS Errors
- Ensure `FRONTEND_URL` in `.env` matches your frontend URL
- For multiple origins, update CORS config in `server.js`

### JWT Token Expired
- Tokens expire based on `JWT_EXPIRE` setting
- Implement refresh token logic for longer sessions
- User must login again after expiration

## Production Deployment

### Checklist

1. ✅ Change `JWT_SECRET` to strong random string
2. ✅ Use production MongoDB URI (MongoDB Atlas)
3. ✅ Set `NODE_ENV=production`
4. ✅ Use HTTPS for API endpoints
5. ✅ Enable rate limiting
6. ✅ Set up logging (Winston, Morgan)
7. ✅ Use process manager (PM2)
8. ✅ Set up monitoring (New Relic, DataDog)

### Deploy to Heroku

```bash
# Install Heroku CLI
heroku login
heroku create your-app-name

# Set environment variables
heroku config:set NODE_ENV=production
heroku config:set MONGODB_URI=your-mongo-atlas-uri
heroku config:set JWT_SECRET=your-secret

# Deploy
git push heroku main
```

### Deploy to AWS/DigitalOcean

```bash
# Install PM2
npm install -g pm2

# Start server
pm2 start src/server.js --name robotics-api

# Save PM2 process
pm2 save
pm2 startup
```

## Next Steps

- [ ] Add email verification
- [ ] Implement refresh tokens
- [ ] Add rate limiting (express-rate-limit)
- [ ] Add API documentation (Swagger)
- [ ] Implement social OAuth (Google, GitHub)
- [ ] Add user profile management
- [ ] Add course enrollment endpoints
- [ ] Add progress tracking endpoints

## Support

For issues or questions:
- Check the troubleshooting section
- Review error logs
- Contact: support@example.com

## License

MIT License
