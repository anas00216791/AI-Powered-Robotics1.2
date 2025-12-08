# Quick Start Guide

Get your backend running in 5 minutes!

## Step 1: Install Dependencies

```bash
cd backend
npm install
```

Wait for installation to complete...

## Step 2: Configure Environment

The `.env` file has been created for you. Update these values:

1. **MongoDB**: Choose one option:

   **Option A - Local MongoDB (Easier for development):**
   ```env
   MONGODB_URI=mongodb://localhost:27017/robotics-course
   ```

   **Option B - MongoDB Atlas (Cloud, Free tier available):**
   - Go to https://www.mongodb.com/atlas
   - Create free account
   - Create cluster
   - Get connection string
   - Update `MONGODB_URI` in `.env`

2. **JWT Secret** (IMPORTANT - Change this!)
   ```env
   JWT_SECRET=your-super-secret-random-string-here
   ```
   Generate random string: https://randomkeygen.com/

3. **Email (Optional for now - needed for password reset)**
   - Use Gmail: Enable 2FA and create App Password
   - Or skip for now (password reset won't work)

## Step 3: Start MongoDB (if using local)

**Windows:**
```bash
# If installed, MongoDB should be running as service
# Or start manually:
mongod
```

**Mac:**
```bash
brew services start mongodb-community
```

**Linux:**
```bash
sudo systemctl start mongod
```

## Step 4: Start the Server

```bash
npm run dev
```

You should see:
```
✅ MongoDB connected successfully
🚀 Server is running on port 5000
📍 API URL: http://localhost:5000/api
```

## Step 5: Test the API

**Health Check:**
```bash
curl http://localhost:5000/api/health
```

**Test Signup:**
```bash
curl -X POST http://localhost:5000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d "{\"fullName\":\"Test User\",\"email\":\"test@example.com\",\"password\":\"Test123456\"}"
```

## Step 6: Connect Frontend

Your frontend is already configured! Just make sure both servers are running:

**Terminal 1 - Backend:**
```bash
cd backend
npm run dev
```

**Terminal 2 - Frontend:**
```bash
cd my-book
npm start
```

Now visit http://localhost:3000/login and try logging in!

## Troubleshooting

### MongoDB Connection Failed
- Check if MongoDB is running: `mongo` or `mongosh`
- Verify connection string in `.env`
- For Atlas: Whitelist your IP in Atlas dashboard

### Port 5000 Already in Use
Update `.env`:
```env
PORT=5001
```
Then update frontend `REACT_APP_API_URL` accordingly.

### Dependencies Installation Error
```bash
rm -rf node_modules package-lock.json
npm install
```

## Next Steps

1. ✅ Backend running
2. ✅ Frontend running
3. ✅ Test login/signup
4. 📧 Configure email (for password reset)
5. 🚀 Deploy to production

## Need Help?

Check the full README.md for detailed documentation!
