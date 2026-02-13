#!/bin/bash

# Build and Deploy Task Management Application to Minikube
# This script builds Docker images and deploys them to minikube

set -e

echo "=========================================="
echo "Task Management - Build and Deploy Script"
echo "=========================================="

# Check if minikube is running
if ! minikube status > /dev/null 2>&1; then
    echo "Error: Minikube is not running. Please start minikube first."
    echo "Run: minikube start"
    exit 1
fi

# Set Docker environment to use minikube's Docker daemon
echo "Setting Docker environment to minikube..."
eval $(minikube docker-env)

# Build Backend Image
echo ""
echo "Building Backend Docker image..."
cd ../backend
docker build -t task-management-backend:latest .
echo "✓ Backend image built successfully"

# Build Frontend Image
echo ""
echo "Building Frontend Docker image..."
cd ../frontend-app
docker build -t task-management-frontend:latest .
echo "✓ Frontend image built successfully"

# Return to helm chart directory
cd ../task-management

# Deploy with Helm
echo ""
echo "Deploying to Kubernetes with Helm..."
helm upgrade --install task-management . \
    --set frontend.image.pullPolicy=Never \
    --set backend.image.pullPolicy=Never \
    --wait \
    --timeout 5m

echo ""
echo "=========================================="
echo "Deployment Complete!"
echo "=========================================="
echo ""
echo "To access your application:"
echo "  Frontend: minikube service task-management-frontend"
echo "  Backend:  minikube service task-management-backend"
echo ""
echo "Or get the URLs:"
echo "  minikube service list"
echo ""
echo "To check pod status:"
echo "  kubectl get pods"
echo ""
