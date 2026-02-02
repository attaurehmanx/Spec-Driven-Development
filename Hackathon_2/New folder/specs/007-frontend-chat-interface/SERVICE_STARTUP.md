# Service Startup Guide

**Feature**: 007-frontend-chat-interface
**Date**: 2026-01-30

## Quick Start Commands

### Option 1: Start Both Services (Recommended)

Open two separate terminal windows:

**Terminal 1 - Backend**:
```bash
cd "Z:\phse 33\backend"
venv\Scripts\activate
uvicorn main:app --reload --port 8000
```

**Terminal 2 - Frontend**:
```bash
cd "Z:\phse 33\frontend-app"
npm run dev
```

### Option 2: Check if Services Are Already Running

**Check Backend**:
```bash
curl http://localhost:8000/health
```
Expected response: `{"status":"ok"}`

**Check Frontend**:
Open browser to: http://localhost:3000
Should see the application homepage.

---

## Detailed Backend Setup

### 1. Navigate to Backend Directory
```bash
cd "Z:\phse 33\backend"
```

### 2. Activate Virtual Environment

**Windows**:
```bash
venv\Scripts\activate
```

**Linux/Mac**:
```bash
source venv/bin/activate
```

### 3. Verify Environment Variables

Check that `.env` file exists with required variables:
```bash
# View .env file (Windows)
type .env

# View .env file (Linux/Mac)
cat .env
```

Required variables:
- `DATABASE_URL` - PostgreSQL connection string
- `BETTER_AUTH_SECRET` - JWT secret key
- `GEMINI_API_KEY` - Google Gemini API key

### 4. Start Backend Server

```bash
uvicorn main:app --reload --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [xxxxx] using StatReload
INFO:     Started server process [xxxxx]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 5. Verify Backend is Running

**Option A: Browser**
- Open: http://localhost:8000/docs
- Should see FastAPI Swagger documentation

**Option B: Command Line**
```bash
curl http://localhost:8000/health
```
Expected: `{"status":"ok"}`

---

## Detailed Frontend Setup

### 1. Navigate to Frontend Directory
```bash
cd "Z:\phse 33\frontend-app"
```

### 2. Verify Environment Variables

Check that `.env.local` file exists:
```bash
# View .env.local file (Windows)
type .env.local

# View .env.local file (Linux/Mac)
cat .env.local
```

Required variables:
- `NEXT_PUBLIC_BACKEND_URL=http://localhost:8000`

### 3. Install Dependencies (if needed)

```bash
npm install
```

### 4. Start Development Server

```bash
npm run dev
```

Expected output:
```
  ▲ Next.js 16.1.1
  - Local:        http://localhost:3000
  - Network:      http://192.168.x.x:3000

 ✓ Ready in 2.5s
```

### 5. Verify Frontend is Running

**Browser**:
- Open: http://localhost:3000
- Should see application homepage
- Check browser console (F12) for errors

---

## Troubleshooting

### Backend Won't Start

**Error: "Address already in use"**
```bash
# Find process using port 8000 (Windows)
netstat -ano | findstr :8000

# Kill process (Windows)
taskkill /PID <process_id> /F

# Find process using port 8000 (Linux/Mac)
lsof -i :8000

# Kill process (Linux/Mac)
kill -9 <process_id>
```

**Error: "No module named 'fastapi'"**
```bash
# Ensure virtual environment is activated
venv\Scripts\activate

# Reinstall dependencies
pip install -r requirements.txt
```

**Error: "Database connection failed"**
- Check `DATABASE_URL` in `.env` file
- Verify PostgreSQL database is running
- Test connection: `psql $DATABASE_URL`

**Error: "GEMINI_API_KEY not found"**
- Check `.env` file has `GEMINI_API_KEY=your-key-here`
- Verify API key is valid at https://makersuite.google.com/app/apikey

### Frontend Won't Start

**Error: "Port 3000 is already in use"**
```bash
# Find process using port 3000 (Windows)
netstat -ano | findstr :3000

# Kill process (Windows)
taskkill /PID <process_id> /F

# Find process using port 3000 (Linux/Mac)
lsof -i :3000

# Kill process (Linux/Mac)
kill -9 <process_id>
```

**Error: "Module not found"**
```bash
# Delete node_modules and reinstall
rm -rf node_modules
npm install
```

**Error: "NEXT_PUBLIC_BACKEND_URL not defined"**
- Create `.env.local` file in `frontend-app/` directory
- Add: `NEXT_PUBLIC_BACKEND_URL=http://localhost:8000`

### Services Running but Chat Not Working

**Check Backend Logs**:
- Look at terminal running backend
- Check for errors when sending chat messages

**Check Frontend Console**:
- Open browser DevTools (F12)
- Go to Console tab
- Look for errors when opening chat or sending messages

**Check Network Requests**:
- Open browser DevTools (F12)
- Go to Network tab
- Send a chat message
- Look for POST request to `/api/{user_id}/chat`
- Check response status (should be 200)
- If 401: Authentication issue (check JWT token)
- If 404: Backend endpoint not found
- If 500: Backend error (check backend logs)

**Check JWT Token**:
- Open browser DevTools (F12)
- Go to Application tab → Local Storage
- Look for `access_token` key
- Should have a long string value (JWT token)
- If missing: Sign out and sign in again

---

## Verification Checklist

Before starting tests, verify:

- [ ] Backend running on http://localhost:8000
- [ ] Backend health check returns `{"status":"ok"}`
- [ ] Backend Swagger docs accessible at http://localhost:8000/docs
- [ ] Frontend running on http://localhost:3000
- [ ] Frontend homepage loads without errors
- [ ] Browser console shows no errors
- [ ] User is signed in (JWT token in localStorage)
- [ ] Dashboard page loads at http://localhost:3000/dashboard
- [ ] Task list is visible on dashboard

---

## Stopping Services

### Stop Backend
- Press `Ctrl+C` in backend terminal
- Deactivate virtual environment: `deactivate`

### Stop Frontend
- Press `Ctrl+C` in frontend terminal

---

## Restart Services

If you need to restart after making changes:

**Backend** (auto-reloads with `--reload` flag):
- No restart needed for code changes
- Restart needed for `.env` changes

**Frontend** (auto-reloads with Next.js):
- No restart needed for most code changes
- Restart needed for `.env.local` changes
- Restart needed for `next.config.js` changes

---

## Production Mode (Optional)

### Backend Production
```bash
cd "Z:\phse 33\backend"
venv\Scripts\activate
uvicorn main:app --host 0.0.0.0 --port 8000
```

### Frontend Production
```bash
cd "Z:\phse 33\frontend-app"
npm run build
npm start
```

---

## Environment Variables Reference

### Backend (.env)
```bash
# Database
DATABASE_URL=postgresql://user:password@host:port/database

# Authentication
BETTER_AUTH_SECRET=your-secret-key-minimum-32-characters

# AI Service
GEMINI_API_KEY=your-gemini-api-key-here

# Server (optional)
PORT=8000
HOST=0.0.0.0
```

### Frontend (.env.local)
```bash
# Backend API URL
NEXT_PUBLIC_BACKEND_URL=http://localhost:8000

# Better Auth (if needed)
BETTER_AUTH_SECRET=your-secret-key-minimum-32-characters
BETTER_AUTH_URL=http://localhost:3000
```

---

## Next Steps

Once both services are running:
1. Sign in to the application
2. Navigate to dashboard
3. Follow the testing guide: `TESTING_GUIDE.md`
