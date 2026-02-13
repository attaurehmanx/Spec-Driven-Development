# Quick Start Guide - Task Management on Minikube

## Prerequisites Check

```bash
# Check minikube
minikube version

# Check helm
helm version

# Check docker
docker --version

# Check kubectl
kubectl version --client
```

## Step-by-Step Deployment

### 1. Start Minikube (if not running)

```bash
minikube start
```

Wait for minikube to fully start (30-45 minutes first time, 1-3 minutes after).

### 2. Configure Docker to Use Minikube

**Windows (PowerShell):**
```powershell
& minikube -p minikube docker-env --shell powershell | Invoke-Expression
```

**Windows (CMD):**
```cmd
@FOR /f "tokens=*" %i IN ('minikube -p minikube docker-env --shell cmd') DO @%i
```

**Linux/Mac:**
```bash
eval $(minikube docker-env)
```

### 3. Build Docker Images

```bash
# Build backend
cd backend
docker build -t task-management-backend:latest .

# Build frontend
cd ../frontend-app
docker build -t task-management-frontend:latest .

# Verify images
docker images | grep task-management
```

### 4. Deploy with Helm

```bash
cd ../task-management

# Install
helm install task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never

# Or use the automated script
# Windows: build-and-deploy.bat
# Linux/Mac: ./build-and-deploy.sh
```

### 5. Verify Deployment

```bash
# Check pods (wait until all are Running)
kubectl get pods

# Check services
kubectl get services

# View logs
kubectl logs -l app=task-management-frontend -f
```

### 6. Access Application

**Option 1: Minikube Service (Recommended)**
```bash
# Frontend
minikube service task-management-frontend

# Backend
minikube service task-management-backend
```

**Option 2: Direct URL**
```bash
# Get minikube IP
minikube ip

# Access:
# Frontend: http://<minikube-ip>:30080
# Backend: http://<minikube-ip>:30081
# API Docs: http://<minikube-ip>:30081/docs
```

**Option 3: Port Forward**
```bash
kubectl port-forward svc/task-management-frontend 3000:3000
kubectl port-forward svc/task-management-backend 8001:8001
```

## Troubleshooting

### Pods Not Starting

```bash
# Check pod status
kubectl get pods

# Describe pod
kubectl describe pod <pod-name>

# View logs
kubectl logs <pod-name>
```

### Image Pull Errors

Make sure you're using minikube's Docker daemon:
```bash
eval $(minikube docker-env)  # Linux/Mac
docker images | grep task-management
```

### Database Connection Issues

```bash
# Check database pod
kubectl logs -l app=task-management-database

# Connect to database
kubectl exec -it <postgres-pod-name> -- psql -U user -d taskdb
```

### Service Not Accessible

```bash
# List all services
minikube service list

# Get service URL
minikube service task-management-frontend --url
```

## Update Application

After making code changes:

```bash
# Rebuild images (with minikube docker-env set)
cd backend
docker build -t task-management-backend:latest .

cd ../frontend-app
docker build -t task-management-frontend:latest .

# Upgrade deployment
cd ../task-management
helm upgrade task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never

# Or restart pods to pull new images
kubectl rollout restart deployment/task-management-frontend
kubectl rollout restart deployment/task-management-backend
```

## Clean Up

```bash
# Uninstall application
helm uninstall task-management

# Delete persistent volume claims
kubectl delete pvc --all

# Stop minikube
minikube stop

# Delete minikube cluster (optional)
minikube delete
```

## Common Commands

```bash
# View all resources
kubectl get all

# Watch pod status
kubectl get pods -w

# View service endpoints
kubectl get endpoints

# Check helm releases
helm list

# Get helm values
helm get values task-management

# View helm manifest
helm get manifest task-management
```
