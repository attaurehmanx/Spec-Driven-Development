# Research: Containerization Strategy

**Feature**: 009-containerization
**Date**: 2026-02-05
**Purpose**: Analyze existing containerization state and identify optimization opportunities

## Executive Summary

The backend already has a functional Dockerfile but requires modifications to align with constitutional principles and feature requirements. The frontend lacks containerization entirely and requires a new multi-stage Dockerfile with nginx. Both containers must be stateless, accept environment-based configuration, and meet size/security targets.

## Existing Backend Dockerfile Analysis

### Current Implementation

**File**: `backend/Dockerfile`

**Base Image**: `python:3.11-slim`
- ✅ Good choice: Smaller than full Python image
- ✅ Meets FR-012 (Python 3.10+)
- ✅ Security updates available

**Security Posture**:
- ✅ Non-root user created (user:1000)
- ✅ Proper file ownership with --chown
- ✅ Health check implemented
- ⚠️ Creates uploads/avatars directory (violates Principle X - stateless)

**Port Configuration**:
- ❌ Current: Port 7860 (Hugging Face Spaces default)
- ✅ Required: Port 8000 (FR-003)
- **Action**: Change EXPOSE and CMD port

**Application Entry Point**:
- ❌ Current: `uvicorn app:app`
- ✅ Required: `uvicorn main:app` (based on main.py existence)
- **Action**: Update CMD to reference correct module

**Environment Variables**:
- ✅ PYTHONUNBUFFERED=1 (proper logging)
- ✅ PYTHONDONTWRITEBYTECODE=1 (no .pyc files)
- ✅ PIP settings for optimization
- ⚠️ No explicit handling of required runtime vars (DATABASE_URL, etc.)

**Health Check**:
- ✅ Implemented with 30s interval
- ✅ Uses Python requests library
- ⚠️ Checks port 7860, needs update to 8000

**Image Size Estimate**:
- Base python:3.11-slim: ~150MB
- Dependencies (FastAPI, SQLModel, etc.): ~200-250MB
- Application code: ~10-20MB
- **Estimated Total**: 360-420MB
- ✅ Under 500MB target (SC-003)

### Required Modifications

1. **Port Update**:
   ```dockerfile
   EXPOSE 8000
   CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
   ```

2. **Remove Stateful Elements**:
   ```dockerfile
   # REMOVE THIS LINE:
   # mkdir -p /app/uploads/avatars && \
   ```

3. **Update Health Check**:
   ```dockerfile
   HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
       CMD python -c "import requests; requests.get('http://localhost:8000/health', timeout=5)" || exit 1
   ```

4. **Document Required Environment Variables**:
   - Add comment block listing required vars
   - No defaults in Dockerfile (Principle XI)

### Optimization Opportunities

**Multi-Stage Build Consideration**:
- Current: Single-stage build
- Benefit: Could separate build dependencies from runtime
- Analysis: FastAPI apps typically don't need build stage unless compiling extensions
- **Recommendation**: Keep single-stage for simplicity, monitor size

**Dependency Optimization**:
- Review requirements.txt for unused packages
- Consider using `pip install --no-deps` for specific packages
- Use `pip list --format=freeze` to verify installed versions

**Layer Caching**:
- ✅ Already optimal: COPY requirements.txt before application code
- Dependencies cached unless requirements.txt changes

## Frontend Containerization Research

### Current State

**Status**: No Dockerfile exists
**Directory**: `frontend-app/` (not `frontend/`)
**Framework**: Next.js 14+ with App Router

### Next.js Build Output Analysis

**Build Command**: `npm run build`
**Output**: `.next/` directory containing:
- Static HTML/CSS/JS files
- Server-side rendering artifacts (if used)
- API routes (if any)

**Deployment Options**:
1. **Node.js Server** (next start): Full Next.js server with SSR
2. **Static Export** (next export): Pure static files for nginx
3. **Standalone Mode**: Minimal Node.js runtime

**Recommendation**: Static export with nginx (smallest image, aligns with stateless principle)

### Multi-Stage Build Strategy

**Stage 1: Builder**
```dockerfile
FROM node:18-alpine AS builder
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY . .
RUN npm run build
```

**Stage 2: Runtime**
```dockerfile
FROM nginx:alpine
COPY --from=builder /app/.next/static /usr/share/nginx/html/_next/static
COPY --from=builder /app/public /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
```

**Size Estimate**:
- nginx:alpine base: ~25MB
- Static build artifacts: ~30-50MB
- nginx.conf: <1KB
- **Estimated Total**: 55-75MB
- ✅ Under 100MB target (SC-004)

### nginx Configuration Requirements

**Routing Needs**:
1. Serve static files from /usr/share/nginx/html
2. Proxy /api/* to backend service
3. Handle Next.js routing (try_files with fallback)
4. Health check endpoint

**Backend Service Discovery**:
- Local Docker: `http://host.docker.internal:8000`
- Docker Compose: `http://todo-backend:8000`
- Kubernetes: `http://todo-backend:8000` (service name)

**Recommendation**: Use environment variable for backend URL, default to Kubernetes service name

## .dockerignore Analysis

### Backend .dockerignore (Existing)

**File**: `backend/.dockerignore`

**Current Content** (needs verification):
```
__pycache__/
*.pyc
*.pyo
*.pyd
.Python
venv/
env/
.env
.git/
.gitignore
*.md
tests/
.pytest_cache/
```

**Verification Needed**:
- ✅ Excludes Python cache files
- ✅ Excludes virtual environments
- ✅ Excludes .env (secrets)
- ✅ Excludes .git
- ⚠️ Check if tests/ should be excluded (depends on whether tests run in container)

**Recommendations**:
- Add `*.log` to exclude log files
- Add `.vscode/`, `.idea/` for IDE files
- Add `__pycache__` at root level

### Frontend .dockerignore (To Create)

**Required Content**:
```
node_modules/
.next/
.git/
.gitignore
*.md
npm-debug.log*
yarn-debug.log*
yarn-error.log*
.env*.local
.vercel
.vscode/
.idea/
coverage/
.DS_Store
```

**Rationale**:
- `node_modules/`: Rebuilt in container (npm ci)
- `.next/`: Build output, created during build
- `.env*.local`: Local secrets, not for container
- IDE files: Not needed in container

## Environment Variables Inventory

### Backend Required Variables

| Variable | Purpose | Example | Source |
|----------|---------|---------|--------|
| DATABASE_URL | Neon PostgreSQL connection | postgresql://user:pass@host/db | Kubernetes Secret |
| GEMINI_API_KEY | Google Gemini API access | AIza... | Kubernetes Secret |
| BETTER_AUTH_SECRET | JWT signing key | random-secret-string | Kubernetes Secret |
| BETTER_AUTH_URL | Auth service URL | https://auth.example.com | ConfigMap |

**Validation Strategy**:
- Check at application startup
- Fail fast if missing critical vars
- Log which vars are missing (without values)

### Frontend Required Variables

**Option 1: Environment Variables**
| Variable | Purpose | Example |
|----------|---------|---------|
| NEXT_PUBLIC_API_URL | Backend API URL | http://todo-backend:8000 |

**Option 2: nginx Proxy (Recommended)**
- No environment variables needed
- nginx.conf hardcodes backend service name
- Simpler deployment, fewer moving parts

**Recommendation**: Use nginx proxy approach for simplicity

## Security Considerations

### Image Scanning Requirements

**Tools**:
- Docker Scout (built into Docker Desktop)
- Trivy (open source)
- Snyk (commercial)

**Scan Targets**:
- Base images (python:3.11-slim, nginx:alpine)
- Installed packages (pip, npm)
- Application dependencies

**Acceptance Criteria** (SC-007):
- Zero critical vulnerabilities
- Zero high vulnerabilities
- Medium/low acceptable with justification

### Non-Root User Implementation

**Backend**:
- ✅ Already implemented (user:1000)
- Verify file permissions for /app

**Frontend**:
- nginx:alpine runs as nginx user by default
- Verify no root processes in container

### Secrets Management

**Constitutional Requirement** (Principle XI):
- ❌ Never hardcode in Dockerfile
- ❌ Never commit to .env files
- ✅ Inject at runtime via docker run -e
- ✅ Use Kubernetes Secrets in production

**Verification**:
- Scan Dockerfiles for hardcoded secrets
- Check .env files are in .gitignore
- Test containers fail gracefully without secrets

## Build Performance Analysis

### Backend Build Time

**Stages**:
1. Pull base image: ~30s (cached after first pull)
2. Install dependencies: ~60-90s (cached if requirements.txt unchanged)
3. Copy application code: ~5s
4. Total: ~95-125s first build, ~10s cached builds

**Optimization**:
- ✅ Already optimal layer ordering
- Consider using pip wheel cache for faster installs

### Frontend Build Time

**Stages**:
1. Pull node:18-alpine: ~20s (cached)
2. npm ci: ~60-120s (cached if package-lock.json unchanged)
3. npm run build: ~60-90s (Next.js build)
4. Pull nginx:alpine: ~10s (cached)
5. Copy artifacts: ~5s
6. Total: ~155-245s first build, ~70-95s cached builds

**Optimization**:
- Use npm ci instead of npm install (faster, deterministic)
- Consider build cache for Next.js
- Parallel builds if multiple containers

**Combined Build Time**: ~250-370s (4-6 minutes)
- ✅ Under 5 minute target (SC-006) with caching
- ⚠️ First build may exceed 5 minutes, acceptable

## Container Runtime Compatibility

### Docker
- ✅ Primary target
- ✅ All features supported
- ✅ Health checks supported

### containerd
- ✅ Kubernetes default runtime
- ✅ OCI-compliant images work
- ⚠️ Health checks handled by Kubernetes probes

### CRI-O
- ✅ Alternative Kubernetes runtime
- ✅ OCI-compliant images work
- ⚠️ Health checks handled by Kubernetes probes

**Recommendation**: Build OCI-compliant images, test with Docker, verify in Kubernetes

## Logging Strategy

### Constitutional Requirement (FR-015)

**Backend**:
- ✅ uvicorn logs to stdout by default
- ✅ FastAPI logs to stdout
- ✅ Python logging configured for stdout

**Frontend**:
- ✅ nginx logs to stdout/stderr by default
- ✅ Access logs: stdout
- ✅ Error logs: stderr

**Verification**:
```bash
docker logs todo-backend
docker logs todo-frontend
```

## Health Check Implementation

### Backend Health Check

**Endpoint**: `/health` (assumed to exist in FastAPI app)
**Method**: GET
**Expected Response**: 200 OK
**Timeout**: 10s
**Interval**: 30s

**Dockerfile Implementation**:
```dockerfile
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import requests; requests.get('http://localhost:8000/health', timeout=5)" || exit 1
```

### Frontend Health Check

**Endpoint**: `/health` (nginx custom endpoint)
**Method**: GET
**Expected Response**: 200 OK with "healthy" text

**nginx.conf Implementation**:
```nginx
location /health {
    access_log off;
    return 200 "healthy\n";
    add_header Content-Type text/plain;
}
```

## Recommendations Summary

### High Priority
1. ✅ Update backend Dockerfile port to 8000
2. ✅ Remove stateful directory creation from backend
3. ✅ Create frontend multi-stage Dockerfile
4. ✅ Create nginx.conf for frontend
5. ✅ Create frontend .dockerignore

### Medium Priority
6. ✅ Verify backend .dockerignore completeness
7. ✅ Document required environment variables
8. ✅ Test health checks for both containers
9. ✅ Run security scans with Gordon

### Low Priority
10. ⚠️ Consider build cache optimization
11. ⚠️ Monitor image sizes, optimize if needed
12. ⚠️ Add container labels for metadata

## Gordon Consultation Strategy

### Consultation Order

1. **Backend Security Scan** (Priority: High)
   - Scan existing Dockerfile
   - Identify vulnerabilities
   - Get remediation recommendations

2. **Frontend Dockerfile Generation** (Priority: High)
   - Generate multi-stage Dockerfile
   - Optimize for size (<100MB)
   - Include best practices

3. **nginx Configuration** (Priority: High)
   - Generate nginx.conf
   - Include reverse proxy
   - Add health check

4. **Final Security Scan** (Priority: Medium)
   - Scan both built images
   - Verify no critical/high vulnerabilities
   - Document findings

## Next Steps

1. Create quickstart.md with build/run instructions
2. Run `/sp.tasks` to generate implementation tasks
3. Consult Gordon for Dockerfile generation
4. Implement tasks in priority order
5. Verify all success criteria
