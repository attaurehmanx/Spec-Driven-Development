# Task Management Helm Chart

This Helm chart deploys the Task Management application (Frontend + Backend + Database) to Kubernetes/Minikube.

## Architecture

- **Frontend**: Next.js application (Port 3000, NodePort 30080)
- **Backend**: FastAPI application (Port 8001, NodePort 30081)
- **Database**: PostgreSQL 15 (Port 5432, ClusterIP)

## Prerequisites

- Minikube installed and running
- Helm 3.x installed
- Docker installed
- kubectl configured

## Quick Start

### 1. Start Minikube

```bash
minikube start
```

### 2. Build and Deploy (Automated)

**Windows:**
```cmd
build-and-deploy.bat
```

**Linux/Mac:**
```bash
chmod +x build-and-deploy.sh
./build-and-deploy.sh
```

### 3. Access the Application

Get service URLs:
```bash
minikube service list
```

Open frontend:
```bash
minikube service task-management-frontend
```

Open backend API docs:
```bash
minikube service task-management-backend
```

Or access directly:
- Frontend: `http://<minikube-ip>:30080`
- Backend: `http://<minikube-ip>:30081`

Get minikube IP:
```bash
minikube ip
```

## Manual Deployment

### 1. Build Docker Images

Set Docker to use minikube's daemon:
```bash
# Linux/Mac
eval $(minikube docker-env)

# Windows (PowerShell)
& minikube -p minikube docker-env --shell powershell | Invoke-Expression

# Windows (CMD)
@FOR /f "tokens=*" %i IN ('minikube -p minikube docker-env --shell cmd') DO @%i
```

Build images:
```bash
# Backend
cd ../backend
docker build -t task-management-backend:latest .

# Frontend
cd ../frontend-app
docker build -t task-management-frontend:latest .
```

### 2. Deploy with Helm

```bash
cd task-management
helm install task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never
```

### 3. Verify Deployment

```bash
# Check pods
kubectl get pods

# Check services
kubectl get services

# View logs
kubectl logs -l app=task-management-backend -f
kubectl logs -l app=task-management-frontend -f
```

## Configuration

Edit `values.yaml` to customize:

- Replica counts
- Resource limits
- Environment variables
- Service types and ports
- Database credentials

## Useful Commands

```bash
# Upgrade deployment
helm upgrade task-management .

# Uninstall
helm uninstall task-management

# Check status
helm status task-management

# View all resources
kubectl get all

# Delete all resources
kubectl delete all --all

# Port forward (alternative access)
kubectl port-forward svc/task-management-frontend 3000:3000
kubectl port-forward svc/task-management-backend 8001:8001
```

## Troubleshooting

### Pods not starting
```bash
kubectl describe pod <pod-name>
kubectl logs <pod-name>
```

### Image pull errors
Ensure you're using minikube's Docker daemon:
```bash
eval $(minikube docker-env)
docker images | grep task-management
```

### Database connection issues
Check database pod:
```bash
kubectl logs -l app=task-management-database
kubectl exec -it <postgres-pod-name> -- psql -U user -d taskdb
```

### Service not accessible
```bash
minikube service list
minikube service task-management-frontend --url
```

## Development

To update the application:

1. Make code changes
2. Rebuild Docker images
3. Upgrade helm release:
```bash
helm upgrade task-management . \
  --set frontend.image.pullPolicy=Never \
  --set backend.image.pullPolicy=Never
```

Or use the automated script again.

## Clean Up

```bash
# Uninstall helm release
helm uninstall task-management

# Delete PVC (if needed)
kubectl delete pvc --all

# Stop minikube
minikube stop

# Delete minikube cluster
minikube delete
```
