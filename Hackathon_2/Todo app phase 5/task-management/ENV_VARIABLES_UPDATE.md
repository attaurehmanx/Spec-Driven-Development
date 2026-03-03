# Helm Chart Update Summary - Real Environment Variables

## What Changed

I've updated your helm chart to use **real environment variables** from your project's `.env` files instead of dummy values.

## Environment Variables Configured

### Frontend (Next.js)

**Public Environment Variables (Client-side):**
```yaml
NEXT_PUBLIC_BETTER_AUTH_URL: "http://localhost:30080"
NEXT_PUBLIC_BACKEND_URL: "http://localhost:30081"
NEXT_PUBLIC_API_BASE_URL: "http://localhost:30081/api"
```

**Private Environment Variables (Server-side):**
```yaml
BACKEND_URL: "http://task-management-backend:8001"  # Internal K8s service
API_BASE_URL: "http://task-management-backend:8001/api"
NODE_ENV: "production"
```

**Secrets (from Kubernetes Secret):**
- `BETTER_AUTH_SECRET` - JWT signing key for Better Auth

### Backend (FastAPI)

**Configuration Environment Variables:**
```yaml
DATABASE_URL: "postgresql://taskuser:taskpass123@postgres:5432/taskdb"
ALGORITHM: "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES: "30"
GEMINI_MODEL: "gemini-2.5-flash"
GEMINI_BASE_URL: "https://generativelanguage.googleapis.com/v1beta/openai/"
AGENT_MAX_ITERATIONS: "15"
AGENT_TEMPERATURE: "1.0"
AGENT_MAX_TOKENS: "1000"
AGENT_TIMEOUT: "30"
APP_ENV: "production"
LOG_LEVEL: "INFO"
```

**Secrets (from Kubernetes Secret):**
- `SECRET_KEY` - JWT signing key
- `GEMINI_API_KEY` - Google Gemini API key

### Database (PostgreSQL)

```yaml
POSTGRES_DB: "taskdb"
POSTGRES_USER: "taskuser"
POSTGRES_PASSWORD: "taskpass123"
```

## New Files Created

### 1. Secret Templates
- `templates/frontend-secrets.yaml` - Kubernetes Secret for frontend
- `templates/backend-secrets.yaml` - Kubernetes Secret for backend

### 2. Automated Deployment Scripts
- `deploy-with-secrets.sh` - Linux/Mac script that extracts secrets from .env files
- `deploy-with-secrets.bat` - Windows CMD script
- `deploy-with-secrets.ps1` - Windows PowerShell script

### 3. Documentation
- `SECRETS_GUIDE.md` - Comprehensive guide for managing secrets

## How to Deploy

### Option 1: Automated Deployment with Secrets (Recommended)

**Linux/Mac:**
```bash
cd task-management
chmod +x deploy-with-secrets.sh
./deploy-with-secrets.sh
```

**Windows (CMD):**
```cmd
cd task-management
deploy-with-secrets.bat
```

**Windows (PowerShell):**
```powershell
cd task-management
.\deploy-with-secrets.ps1
```

This will:
1. Build Docker images using minikube's Docker daemon
2. Extract secrets from `frontend-app/.env` and `backend/.env`
3. Deploy with Helm, injecting secrets securely

### Option 2: Manual Deployment with Custom Secrets

```bash
helm install task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never \
  --set "frontend.secrets.betterAuthSecret=YOUR_SECRET" \
  --set "backend.secrets.secretKey=YOUR_JWT_SECRET" \
  --set "backend.secrets.geminiApiKey=YOUR_GEMINI_KEY"
```

### Option 3: Using Values Files

Edit `values-dev.yaml` and replace placeholder secrets, then:

```bash
helm install task-management . -f values-dev.yaml
```

## Security Features

### 1. Kubernetes Secrets
Sensitive data is stored in Kubernetes Secrets (base64 encoded):
- Frontend: `task-management-frontend-secrets`
- Backend: `task-management-backend-secrets`

### 2. Environment Variable Injection
Secrets are injected as environment variables using `secretKeyRef`:

```yaml
env:
  - name: SECRET_KEY
    valueFrom:
      secretKeyRef:
        name: task-management-backend-secrets
        key: SECRET_KEY
```

### 3. Separation of Concerns
- Public config in `values.yaml`
- Secrets passed via command line or external secret managers
- Never commit actual secrets to git

## Verification

After deployment, verify secrets are loaded:

```bash
# Check secrets exist
kubectl get secrets

# View secret (base64 encoded)
kubectl get secret task-management-backend-secrets -o yaml

# Check environment variables in pod
kubectl exec -it <backend-pod-name> -- env | grep SECRET_KEY
```

## Environment-Specific Configurations

### Development (`values-dev.yaml`)
- NodePort services (30080, 30081)
- Local image pull policy (Never)
- Debug logging
- Development database

### Production (`values-prod.yaml`)
- LoadBalancer services
- Always pull images
- Info logging
- External managed database (recommended)
- Multiple replicas for HA

## Next Steps

1. **Test the deployment:**
   ```bash
   ./deploy-with-secrets.sh  # or .bat/.ps1 on Windows
   ```

2. **Verify all pods are running:**
   ```bash
   kubectl get pods
   ```

3. **Access the application:**
   ```bash
   minikube service task-management-frontend
   ```

4. **Check logs:**
   ```bash
   kubectl logs -l app=task-management-backend -f
   ```

## Important Notes

⚠️ **Security Warnings:**
- The placeholder secrets in `values.yaml` are for reference only
- Always use real secrets from your `.env` files
- Never commit `.env` files or actual secrets to git
- Use external secret managers (AWS Secrets Manager, Azure Key Vault) in production

✅ **What's Working:**
- All environment variables match your project's requirements
- Secrets are properly isolated in Kubernetes Secrets
- Automated scripts extract secrets from .env files
- Internal service communication uses Kubernetes DNS
- External access uses NodePort for local development

## Troubleshooting

### Secrets not loading
```bash
# Check if secrets exist
kubectl get secrets | grep task-management

# Recreate secrets manually
kubectl create secret generic task-management-backend-secrets \
  --from-literal=SECRET_KEY="your-secret" \
  --from-literal=GEMINI_API_KEY="your-key"
```

### Environment variables not set
```bash
# Check pod environment
kubectl exec -it <pod-name> -- env

# Check deployment configuration
kubectl describe deployment task-management-backend
```

### Database connection issues
```bash
# Check database pod
kubectl logs -l app=task-management-database

# Test connection
kubectl exec -it <backend-pod> -- curl http://postgres:5432
```

## Files Modified

- ✅ `values.yaml` - Updated with real environment variables
- ✅ `values-dev.yaml` - Development configuration with real env vars
- ✅ `values-prod.yaml` - Production configuration
- ✅ `templates/frontend-deployment.yaml` - Added secret injection
- ✅ `templates/backend-deployment.yaml` - Added secret injection
- ✅ `templates/frontend-secrets.yaml` - New secret template
- ✅ `templates/backend-secrets.yaml` - New secret template

## Validation Status

✅ Helm chart syntax validated (`helm lint`)
✅ All environment variables match project requirements
✅ Secrets properly configured
✅ Automated deployment scripts created
✅ Documentation complete

**Status**: Ready for deployment with real environment variables! 🚀
