# Hugging Face Spaces Deployment Guide

## 📦 Project Structure

The backend has been organized for clean deployment:

```
backend/
├── api/                    # API routes (auth, tasks, chat, users)
├── models/                 # SQLModel data models
├── services/              # Business logic layer
├── database/              # Database session and migrations
├── middleware/            # Authentication middleware
├── utils/                 # Utility functions (JWT, logging, validation)
├── config/                # Application configuration
├── uploads/               # Static files (avatars)
├── main.py                # Main FastAPI application
├── app.py                 # Hugging Face Spaces entry point
├── requirements.txt       # Python dependencies
├── Dockerfile             # Container configuration
├── .dockerignore          # Files to exclude from Docker build
├── .env.example           # Environment variables template
│
├── tests/                 # Test files (excluded from deployment)
├── docs/                  # Documentation (excluded from deployment)
└── scripts/               # Utility scripts (excluded from deployment)
```

## 🚀 Deployment Steps

### 1. Create a New Hugging Face Space

1. Go to [Hugging Face Spaces](https://huggingface.co/spaces)
2. Click "Create new Space"
3. Choose:
   - **Space name**: `your-task-management-api`
   - **License**: Apache 2.0 (or your preference)
   - **Space SDK**: Docker
   - **Visibility**: Public or Private

### 2. Configure Environment Variables (Secrets)

In your Space settings, add these secrets:

#### Required Secrets:

```bash
# Database (Neon PostgreSQL)
DATABASE_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require

# JWT Authentication
JWT_SECRET_KEY=your-super-secret-jwt-key-min-32-characters-long
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Better Auth Integration
BETTER_AUTH_SECRET=your-better-auth-secret-key-here

# AI/LLM (Google Gemini)
GEMINI_API_KEY=your-google-gemini-api-key-here
```

#### Optional Secrets:

```bash
APP_ENV=production
LOG_LEVEL=INFO
```

### 3. Push Code to Hugging Face

#### Option A: Using Git

```bash
# Clone your Space repository
git clone https://huggingface.co/spaces/YOUR_USERNAME/your-task-management-api
cd your-task-management-api

# Copy backend files
cp -r /path/to/backend/* .

# Commit and push
git add .
git commit -m "Initial deployment"
git push
```

#### Option B: Using Hugging Face Hub

```bash
# Install huggingface_hub
pip install huggingface_hub

# Login
huggingface-cli login

# Upload files
huggingface-cli upload YOUR_USERNAME/your-task-management-api ./backend --repo-type=space
```

### 4. Verify Deployment

Once deployed, your API will be available at:
```
https://YOUR_USERNAME-your-task-management-api.hf.space
```

Test the health endpoint:
```bash
curl https://YOUR_USERNAME-your-task-management-api.hf.space/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "task-management-auth-api",
  "timestamp": "2024-01-31T12:00:00.000000"
}
```

## 🔧 Configuration Details

### Dockerfile Optimizations

The Dockerfile includes:
- **Python 3.11-slim**: Smaller image size, better performance
- **Non-root user**: Enhanced security
- **Layer caching**: Faster rebuilds
- **Health check**: Automatic container health monitoring
- **Port 7860**: Hugging Face Spaces default port

### .dockerignore

Excludes from Docker build:
- Test files and test directories
- Documentation files
- Utility scripts
- Python cache files
- Virtual environments
- Git repository
- Environment files (secrets set via Spaces UI)

### CORS Configuration

Update `main.py` to allow your frontend domain:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://your-frontend-domain.com",  # Add your frontend URL
        "https://your-frontend.vercel.app"   # If using Vercel
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## 📊 Database Setup (Neon PostgreSQL)

### 1. Create Neon Database

1. Go to [Neon Console](https://console.neon.tech)
2. Create a new project
3. Copy the connection string
4. Add it to Hugging Face Spaces secrets as `DATABASE_URL`

### 2. Database Migrations

The application automatically creates tables on startup via `database/migrations.py`:
- Users table
- Tasks table
- Conversations table
- Messages table
- Indexes and foreign keys

## 🔐 Security Checklist

- [ ] All secrets configured in Hugging Face Spaces (not in code)
- [ ] JWT_SECRET_KEY is at least 32 characters
- [ ] DATABASE_URL uses SSL mode (`?sslmode=require`)
- [ ] CORS origins restricted to your frontend domains
- [ ] Non-root user in Docker container
- [ ] .env file excluded from git (.gitignore)

## 🧪 Testing the Deployment

### 1. Health Check
```bash
curl https://YOUR_SPACE.hf.space/health
```

### 2. API Documentation
Visit: `https://YOUR_SPACE.hf.space/docs`

### 3. Test Authentication
```bash
# Sign up
curl -X POST https://YOUR_SPACE.hf.space/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test123!","name":"Test User"}'

# Sign in
curl -X POST https://YOUR_SPACE.hf.space/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test123!"}'
```

## 🐛 Troubleshooting

### Container Fails to Start

Check logs in Hugging Face Spaces:
1. Go to your Space
2. Click "Logs" tab
3. Look for error messages

Common issues:
- Missing environment variables
- Invalid DATABASE_URL
- Port conflicts (ensure using 7860)

### Database Connection Errors

Verify:
- DATABASE_URL format: `postgresql://user:pass@host/db?sslmode=require`
- Neon database is active (not paused)
- IP allowlist includes Hugging Face (or set to allow all)

### CORS Errors

Update `main.py` CORS middleware to include your frontend domain.

## 📈 Monitoring

### Application Logs

View logs in Hugging Face Spaces dashboard:
- Startup logs
- Request logs
- Error logs

### Health Endpoint

Monitor: `https://YOUR_SPACE.hf.space/health`

Returns:
- `status`: "healthy"
- `service`: "task-management-auth-api"
- `timestamp`: Current UTC time

## 🔄 Updates and Redeployment

To update your deployment:

```bash
# Make changes locally
git add .
git commit -m "Update: description of changes"
git push

# Hugging Face automatically rebuilds and redeploys
```

## 📝 Environment Variables Reference

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| DATABASE_URL | Yes | Neon PostgreSQL connection string | `postgresql://user:pass@host/db` |
| JWT_SECRET_KEY | Yes | Secret key for JWT signing (32+ chars) | `your-secret-key-here` |
| JWT_ALGORITHM | Yes | JWT algorithm | `HS256` |
| ACCESS_TOKEN_EXPIRE_MINUTES | Yes | Token expiration time | `30` |
| BETTER_AUTH_SECRET | Yes | Better Auth secret key | `your-auth-secret` |
| GEMINI_API_KEY | Yes | Google Gemini API key | `AIza...` |
| APP_ENV | No | Application environment | `production` |
| LOG_LEVEL | No | Logging level | `INFO` |

## 🎯 Next Steps

1. ✅ Deploy backend to Hugging Face Spaces
2. ⬜ Deploy frontend to Vercel/Netlify
3. ⬜ Update frontend API URL to point to Hugging Face Space
4. ⬜ Test end-to-end authentication flow
5. ⬜ Configure custom domain (optional)

## 📚 Additional Resources

- [Hugging Face Spaces Documentation](https://huggingface.co/docs/hub/spaces)
- [Docker Spaces Guide](https://huggingface.co/docs/hub/spaces-sdks-docker)
- [Neon PostgreSQL Documentation](https://neon.tech/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com)

---

**Note**: This backend is designed to work with the Next.js frontend using Better Auth for authentication. Ensure both services are configured with matching JWT secrets and CORS settings.
