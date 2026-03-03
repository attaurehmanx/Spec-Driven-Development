# Quickstart: Container Build and Run Guide

**Feature**: 009-containerization
**Date**: 2026-02-05
**Purpose**: Step-by-step guide for building and running containerized Todo AI Chatbot

## Prerequisites

- Docker Desktop installed and running
- Access to Neon PostgreSQL database
- Environment variables configured (see below)
- Backend and frontend applications functional

## Environment Setup

### Required Environment Variables

Create a `.env` file in the project root (DO NOT commit to git):

```bash
# Database
DATABASE_URL=postgresql://user:password@host.region.neon.tech/dbname?sslmode=require

# AI Service
GEMINI_API_KEY=your_gemini_api_key_here

# Authentication
BETTER_AUTH_SECRET=your_random_secret_string_here
BETTER_AUTH_URL=http://localhost:3000
```

**Security Note**: Never commit `.env` files. Ensure `.env` is in `.gitignore`.

## Backend Container

### Build Backend Image

```bash
# Navigate to backend directory
cd backend

# Build the image
docker build -t todo-backend:latest .

# Verify image was created
docker images | grep todo-backend
```

**Expected Output**:
```
todo-backend    latest    abc123def456    2 minutes ago    420MB
```

### Run Backend Container

```bash
# Run with environment variables
docker run -d \
  --name todo-backend \
  -p 8000:8000 \
  -e DATABASE_URL="${DATABASE_URL}" \
  -e GEMINI_API_KEY="${GEMINI_API_KEY}" \
  -e BETTER_AUTH_SECRET="${BETTER_AUTH_SECRET}" \
  todo-backend:latest

# Check container status
docker ps | grep todo-backend

# View logs
docker logs todo-backend

# Follow logs in real-time
docker logs -f todo-backend
```

### Verify Backend Health

```bash
# Health check endpoint
curl http://localhost:8000/health

# Expected response: {"status": "healthy"} or similar

# Test API endpoint
curl http://localhost:8000/api/tasks
```

### Backend Troubleshooting

**Container won't start**:
```bash
# Check logs for errors
docker logs todo-backend

# Common issues:
# - Missing environment variables
# - Database connection failed
# - Port 8000 already in use
```

**Fix port conflict**:
```bash
# Stop conflicting container
docker stop $(docker ps -q --filter "publish=8000")

# Or use different port
docker run -p 8001:8000 ...
```

**Database connection issues**:
```bash
# Verify DATABASE_URL is correct
docker exec todo-backend env | grep DATABASE_URL

# Test database connection from container
docker exec -it todo-backend python -c "from database.connection import engine; print(engine.url)"
```

## Frontend Container

### Build Frontend Image

```bash
# Navigate to frontend directory
cd frontend-app

# Build the image (multi-stage build)
docker build -t todo-frontend:latest .

# Verify image was created
docker images | grep todo-frontend
```

**Expected Output**:
```
todo-frontend    latest    xyz789abc123    3 minutes ago    75MB
```

**Note**: First build may take 3-5 minutes. Subsequent builds are faster due to layer caching.

### Run Frontend Container

**Option 1: Standalone (for testing)**
```bash
docker run -d \
  --name todo-frontend \
  -p 80:80 \
  todo-frontend:latest

# Access at http://localhost
```

**Option 2: Linked to Backend**
```bash
docker run -d \
  --name todo-frontend \
  -p 80:80 \
  --link todo-backend:todo-backend \
  todo-frontend:latest

# Access at http://localhost
# API calls to /api/* will proxy to backend
```

### Verify Frontend Health

```bash
# Health check endpoint
curl http://localhost/health

# Expected response: healthy

# Test static file serving
curl http://localhost

# Test API proxy
curl http://localhost/api/health
```

### Frontend Troubleshooting

**Container won't start**:
```bash
# Check logs
docker logs todo-frontend

# Common issues:
# - Port 80 requires admin/sudo on some systems
# - nginx configuration error
```

**Use alternative port**:
```bash
# Run on port 8080 instead
docker run -p 8080:80 todo-frontend:latest

# Access at http://localhost:8080
```

**API proxy not working**:
```bash
# Check nginx configuration
docker exec todo-frontend cat /etc/nginx/conf.d/default.conf

# Check backend is reachable
docker exec todo-frontend ping todo-backend

# View nginx logs
docker logs todo-frontend
```

## Running Both Containers Together

### Using Docker CLI

```bash
# Start backend first
docker run -d \
  --name todo-backend \
  -p 8000:8000 \
  -e DATABASE_URL="${DATABASE_URL}" \
  -e GEMINI_API_KEY="${GEMINI_API_KEY}" \
  -e BETTER_AUTH_SECRET="${BETTER_AUTH_SECRET}" \
  todo-backend:latest

# Wait for backend to be healthy
sleep 5
curl http://localhost:8000/health

# Start frontend linked to backend
docker run -d \
  --name todo-frontend \
  -p 80:80 \
  --link todo-backend:todo-backend \
  todo-frontend:latest

# Verify both running
docker ps
```

### Using Docker Compose (Optional)

Create `docker-compose.yml` in project root:

```yaml
version: '3.8'

services:
  backend:
    image: todo-backend:latest
    container_name: todo-backend
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=${DATABASE_URL}
      - GEMINI_API_KEY=${GEMINI_API_KEY}
      - BETTER_AUTH_SECRET=${BETTER_AUTH_SECRET}
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 5s

  frontend:
    image: todo-frontend:latest
    container_name: todo-frontend
    ports:
      - "80:80"
    depends_on:
      backend:
        condition: service_healthy
    links:
      - backend:todo-backend
```

**Run with Docker Compose**:
```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop all services
docker-compose down
```

## Container Management

### Stop Containers

```bash
# Stop individual container
docker stop todo-backend
docker stop todo-frontend

# Stop all containers
docker stop $(docker ps -q)
```

### Remove Containers

```bash
# Remove stopped containers
docker rm todo-backend
docker rm todo-frontend

# Force remove running containers
docker rm -f todo-backend todo-frontend
```

### Restart Containers

```bash
# Restart individual container
docker restart todo-backend

# Restart all containers
docker restart $(docker ps -q)
```

### View Container Logs

```bash
# View logs
docker logs todo-backend
docker logs todo-frontend

# Follow logs in real-time
docker logs -f todo-backend

# View last 100 lines
docker logs --tail 100 todo-backend

# View logs with timestamps
docker logs -t todo-backend
```

### Inspect Containers

```bash
# View container details
docker inspect todo-backend

# View container stats (CPU, memory)
docker stats todo-backend

# Execute command in running container
docker exec -it todo-backend bash

# View container processes
docker top todo-backend
```

## Image Management

### List Images

```bash
# List all images
docker images

# List specific images
docker images | grep todo-
```

### Remove Images

```bash
# Remove specific image
docker rmi todo-backend:latest

# Remove all unused images
docker image prune -a
```

### Tag Images

```bash
# Tag for registry
docker tag todo-backend:latest myregistry.com/todo-backend:v1.0.0

# Tag as latest
docker tag todo-backend:abc123 todo-backend:latest
```

## Testing Checklist

### Backend Container Tests

- [ ] Container builds successfully
- [ ] Container starts within 10 seconds
- [ ] Health check endpoint responds
- [ ] API endpoints accessible on port 8000
- [ ] Database connection successful
- [ ] Environment variables loaded correctly
- [ ] Logs visible via `docker logs`
- [ ] Container restarts without data loss
- [ ] Image size under 500MB

### Frontend Container Tests

- [ ] Container builds successfully
- [ ] Container starts within 5 seconds
- [ ] Health check endpoint responds
- [ ] Static files served correctly
- [ ] API proxy works (/api/* → backend)
- [ ] Application loads in browser
- [ ] Logs visible via `docker logs`
- [ ] Image size under 100MB

### Integration Tests

- [ ] Frontend can reach backend via proxy
- [ ] User can log in through frontend
- [ ] Tasks can be created/read/updated/deleted
- [ ] Chat interface works
- [ ] Both containers can restart independently
- [ ] No data loss after container restart

## Performance Benchmarks

### Build Times

```bash
# Measure backend build time
time docker build -t todo-backend:latest ./backend

# Measure frontend build time
time docker build -t todo-frontend:latest ./frontend-app
```

**Expected**:
- Backend: 2-3 minutes (first build), 10-30 seconds (cached)
- Frontend: 3-5 minutes (first build), 1-2 minutes (cached)

### Startup Times

```bash
# Measure backend startup
time docker run --rm todo-backend:latest

# Measure frontend startup
time docker run --rm todo-frontend:latest
```

**Expected**:
- Backend: <10 seconds (SC-001)
- Frontend: <5 seconds (SC-002)

### Image Sizes

```bash
# Check image sizes
docker images | grep todo-

# Expected:
# todo-backend:latest    ~400-450MB (target: <500MB)
# todo-frontend:latest   ~70-90MB (target: <100MB)
```

## Security Verification

### Scan for Vulnerabilities

```bash
# Using Docker Scout (built-in)
docker scout cves todo-backend:latest
docker scout cves todo-frontend:latest

# Using Trivy (if installed)
trivy image todo-backend:latest
trivy image todo-frontend:latest
```

**Acceptance Criteria**: Zero critical or high-severity vulnerabilities (SC-007)

### Verify Non-Root User

```bash
# Check backend user
docker run --rm todo-backend:latest whoami
# Expected: user (not root)

# Check frontend user
docker run --rm todo-frontend:latest whoami
# Expected: nginx (not root)
```

### Verify No Hardcoded Secrets

```bash
# Inspect image layers
docker history todo-backend:latest
docker history todo-frontend:latest

# Search for secrets in image
docker run --rm todo-backend:latest env | grep -i secret
# Should show no hardcoded values
```

## Common Issues and Solutions

### Issue: "Port already in use"

**Solution**:
```bash
# Find process using port
lsof -i :8000  # macOS/Linux
netstat -ano | findstr :8000  # Windows

# Stop conflicting container
docker stop $(docker ps -q --filter "publish=8000")
```

### Issue: "Cannot connect to database"

**Solution**:
```bash
# Verify DATABASE_URL is correct
echo $DATABASE_URL

# Test connection from host
psql $DATABASE_URL

# Check container can reach database
docker exec todo-backend ping host.docker.internal
```

### Issue: "Frontend can't reach backend"

**Solution**:
```bash
# Verify backend is running
docker ps | grep todo-backend

# Check link is established
docker inspect todo-frontend | grep -A 10 Links

# Test from frontend container
docker exec todo-frontend curl http://todo-backend:8000/health
```

### Issue: "Build fails with 'no space left on device'"

**Solution**:
```bash
# Clean up Docker resources
docker system prune -a

# Remove unused volumes
docker volume prune

# Check disk space
df -h
```

## Next Steps

After verifying containers work locally:

1. **Kubernetes Deployment** (Feature 010): Create K8s manifests
2. **Helm Charts** (Feature 011): Package for deployment
3. **CI/CD Integration**: Automate builds
4. **Registry Setup**: Push images to registry
5. **Production Deployment**: Deploy to Minikube

## Additional Resources

- [Docker Documentation](https://docs.docker.com/)
- [Dockerfile Best Practices](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)
- [Multi-Stage Builds](https://docs.docker.com/build/building/multi-stage/)
- [Docker Security](https://docs.docker.com/engine/security/)
