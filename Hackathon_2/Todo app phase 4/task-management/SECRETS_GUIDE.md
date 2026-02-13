# Secrets Management Guide

## Overview

This helm chart uses Kubernetes Secrets to manage sensitive data like API keys, JWT secrets, and database passwords. This guide explains how to properly configure secrets for your deployment.

## Environment Variables Structure

### Frontend Environment Variables

**Public (Client-side):**
- `NEXT_PUBLIC_BETTER_AUTH_URL` - Better Auth URL (accessible from browser)
- `NEXT_PUBLIC_BACKEND_URL` - Backend API URL (accessible from browser)
- `NEXT_PUBLIC_API_BASE_URL` - API base URL (accessible from browser)

**Private (Server-side):**
- `BACKEND_URL` - Internal Kubernetes service URL for backend
- `API_BASE_URL` - Internal Kubernetes API base URL
- `BETTER_AUTH_SECRET` - **SECRET** - JWT signing key for Better Auth

### Backend Environment Variables

**Configuration:**
- `DATABASE_URL` - PostgreSQL connection string
- `ALGORITHM` - JWT algorithm (HS256)
- `ACCESS_TOKEN_EXPIRE_MINUTES` - Token expiration time
- `GEMINI_MODEL` - AI model name
- `GEMINI_BASE_URL` - Gemini API base URL
- `AGENT_MAX_ITERATIONS` - Max agent iterations
- `AGENT_TEMPERATURE` - AI temperature setting
- `AGENT_MAX_TOKENS` - Max tokens per request
- `AGENT_TIMEOUT` - Request timeout
- `APP_ENV` - Application environment
- `LOG_LEVEL` - Logging level

**Secrets:**
- `SECRET_KEY` - **SECRET** - JWT signing key
- `GEMINI_API_KEY` - **SECRET** - Google Gemini API key

### Database Environment Variables

- `POSTGRES_DB` - Database name
- `POSTGRES_USER` - Database user
- `POSTGRES_PASSWORD` - **SECRET** - Database password

## Setting Up Secrets

### Method 1: Using Your .env Files (Recommended for Development)

Create secrets from your existing .env files:

```bash
# Frontend secrets
kubectl create secret generic task-management-frontend-secrets \
  --from-literal=BETTER_AUTH_SECRET="$(grep BETTER_AUTH_SECRET frontend-app/.env | cut -d '=' -f2)"

# Backend secrets
kubectl create secret generic task-management-backend-secrets \
  --from-literal=SECRET_KEY="$(grep JWT_SECRET_KEY backend/.env | cut -d '=' -f2)" \
  --from-literal=GEMINI_API_KEY="$(grep GEMINI_API_KEY backend/.env | cut -d '=' -f2)"
```

Then deploy without secrets in values.yaml:

```bash
helm install task-management . \
  --set frontend.secrets.betterAuthSecret="" \
  --set backend.secrets.secretKey="" \
  --set backend.secrets.geminiApiKey=""
```

### Method 2: Using Helm Values (Quick but Less Secure)

Edit `values.yaml` or `values-dev.yaml` and replace placeholder secrets:

```yaml
frontend:
  secrets:
    betterAuthSecret: "your-actual-secret-from-env-file"

backend:
  secrets:
    secretKey: "your-actual-jwt-secret-from-env-file"
    geminiApiKey: "your-actual-gemini-key-from-env-file"
```

Then deploy:

```bash
helm install task-management . -f values-dev.yaml
```

### Method 3: Using Helm --set Flags (Most Secure)

Pass secrets via command line (they won't be stored in files):

```bash
helm install task-management . \
  --set frontend.secrets.betterAuthSecret="your-secret-here" \
  --set backend.secrets.secretKey="your-jwt-secret-here" \
  --set backend.secrets.geminiApiKey="your-gemini-key-here"
```

### Method 4: Using External Secrets Operator (Production)

For production, use External Secrets Operator to sync from AWS Secrets Manager, Azure Key Vault, or Google Secret Manager.

## Quick Setup Script

Create a file `setup-secrets.sh`:

```bash
#!/bin/bash

# Load secrets from .env files
FRONTEND_ENV="../frontend-app/.env"
BACKEND_ENV="../backend/.env"

# Extract secrets
BETTER_AUTH_SECRET=$(grep BETTER_AUTH_SECRET $FRONTEND_ENV | cut -d '=' -f2)
JWT_SECRET=$(grep JWT_SECRET_KEY $BACKEND_ENV | cut -d '=' -f2)
GEMINI_KEY=$(grep GEMINI_API_KEY $BACKEND_ENV | cut -d '=' -f2)

# Deploy with secrets
helm upgrade --install task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never \
  --set frontend.secrets.betterAuthSecret="$BETTER_AUTH_SECRET" \
  --set backend.secrets.secretKey="$JWT_SECRET" \
  --set backend.secrets.geminiApiKey="$GEMINI_KEY"

echo "Deployment complete with secrets from .env files"
```

Make it executable:

```bash
chmod +x setup-secrets.sh
./setup-secrets.sh
```

## Windows PowerShell Script

Create `setup-secrets.ps1`:

```powershell
# Load secrets from .env files
$frontendEnv = Get-Content "../frontend-app/.env"
$backendEnv = Get-Content "../backend/.env"

# Extract secrets
$betterAuthSecret = ($frontendEnv | Select-String "BETTER_AUTH_SECRET=").ToString().Split("=")[1]
$jwtSecret = ($backendEnv | Select-String "JWT_SECRET_KEY=").ToString().Split("=")[1]
$geminiKey = ($backendEnv | Select-String "GEMINI_API_KEY=").ToString().Split("=")[1]

# Deploy with secrets
helm upgrade --install task-management . `
  --set frontend.image.pullPolicy=Never `
  --set backend.image.pullPolicy=Never `
  --set "frontend.secrets.betterAuthSecret=$betterAuthSecret" `
  --set "backend.secrets.secretKey=$jwtSecret" `
  --set "backend.secrets.geminiApiKey=$geminiKey"

Write-Host "Deployment complete with secrets from .env files"
```

Run it:

```powershell
.\setup-secrets.ps1
```

## Verifying Secrets

Check if secrets are created:

```bash
kubectl get secrets
kubectl describe secret task-management-frontend-secrets
kubectl describe secret task-management-backend-secrets
```

View secret values (base64 encoded):

```bash
kubectl get secret task-management-backend-secrets -o yaml
```

Decode a secret:

```bash
kubectl get secret task-management-backend-secrets -o jsonpath='{.data.SECRET_KEY}' | base64 -d
```

## Updating Secrets

### Update via kubectl

```bash
kubectl create secret generic task-management-backend-secrets \
  --from-literal=SECRET_KEY="new-secret-key" \
  --from-literal=GEMINI_API_KEY="new-api-key" \
  --dry-run=client -o yaml | kubectl apply -f -
```

### Update via Helm

```bash
helm upgrade task-management . \
  --set backend.secrets.secretKey="new-secret-key" \
  --reuse-values
```

After updating secrets, restart pods:

```bash
kubectl rollout restart deployment/task-management-frontend
kubectl rollout restart deployment/task-management-backend
```

## Security Best Practices

1. **Never commit secrets to git**
   - Add `.env` files to `.gitignore`
   - Use placeholder values in `values.yaml`

2. **Use different secrets for each environment**
   - Development: Simple secrets for testing
   - Production: Strong, randomly generated secrets

3. **Rotate secrets regularly**
   - Change JWT secrets every 90 days
   - Rotate API keys when team members leave

4. **Use RBAC to restrict secret access**
   ```bash
   kubectl create role secret-reader --verb=get --resource=secrets
   ```

5. **Enable secret encryption at rest**
   - Configure Kubernetes to encrypt secrets in etcd

6. **Use External Secrets in production**
   - AWS Secrets Manager
   - Azure Key Vault
   - Google Secret Manager
   - HashiCorp Vault

## Troubleshooting

### Pods failing with "secret not found"

Check if secrets exist:
```bash
kubectl get secrets | grep task-management
```

Create missing secrets manually:
```bash
kubectl create secret generic task-management-backend-secrets \
  --from-literal=SECRET_KEY="temp-secret" \
  --from-literal=GEMINI_API_KEY="temp-key"
```

### Environment variables not loading

Check pod environment:
```bash
kubectl exec -it <pod-name> -- env | grep SECRET
```

Describe pod to see secret mounts:
```bash
kubectl describe pod <pod-name>
```

### Secret values not updating

Delete and recreate the secret:
```bash
kubectl delete secret task-management-backend-secrets
kubectl create secret generic task-management-backend-secrets \
  --from-literal=SECRET_KEY="new-value"
```

Restart the deployment:
```bash
kubectl rollout restart deployment/task-management-backend
```

## Example: Complete Setup from .env Files

```bash
# 1. Ensure minikube is running
minikube start

# 2. Set Docker environment
eval $(minikube docker-env)

# 3. Build images
cd ../backend && docker build -t task-management-backend:latest .
cd ../frontend-app && docker build -t task-management-frontend:latest .

# 4. Extract secrets from .env files
cd ../task-management
BETTER_AUTH_SECRET=$(grep BETTER_AUTH_SECRET ../frontend-app/.env | cut -d '=' -f2)
JWT_SECRET=$(grep JWT_SECRET_KEY ../backend/.env | cut -d '=' -f2)
GEMINI_KEY=$(grep GEMINI_API_KEY ../backend/.env | cut -d '=' -f2)

# 5. Deploy with secrets
helm install task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never \
  --set "frontend.secrets.betterAuthSecret=$BETTER_AUTH_SECRET" \
  --set "backend.secrets.secretKey=$JWT_SECRET" \
  --set "backend.secrets.geminiApiKey=$GEMINI_KEY"

# 6. Verify deployment
kubectl get pods
kubectl get secrets

# 7. Access application
minikube service task-management-frontend
```
