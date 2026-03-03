#!/bin/bash

# Automated deployment script with secrets from .env files
# This script extracts secrets from your .env files and deploys to minikube

set -e

echo "=========================================="
echo "Task Management - Automated Deployment"
echo "with Secrets from .env Files"
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

# Extract secrets from .env files
echo ""
echo "Extracting secrets from .env files..."

FRONTEND_ENV="../frontend-app/.env"
BACKEND_ENV="../backend/.env"

if [ ! -f "$FRONTEND_ENV" ]; then
    echo "Error: Frontend .env file not found at $FRONTEND_ENV"
    exit 1
fi

if [ ! -f "$BACKEND_ENV" ]; then
    echo "Error: Backend .env file not found at $BACKEND_ENV"
    exit 1
fi

# Extract secrets (handle quotes and spaces)
BETTER_AUTH_SECRET=$(grep "^BETTER_AUTH_SECRET=" "$FRONTEND_ENV" | cut -d '=' -f2- | tr -d '"' | tr -d "'")
JWT_SECRET=$(grep "^JWT_SECRET_KEY=" "$BACKEND_ENV" | cut -d '=' -f2- | tr -d '"' | tr -d "'")
GEMINI_KEY=$(grep "^GEMINI_API_KEY=" "$BACKEND_ENV" | cut -d '=' -f2- | tr -d '"' | tr -d "'")

# Validate secrets were extracted
if [ -z "$BETTER_AUTH_SECRET" ]; then
    echo "Warning: BETTER_AUTH_SECRET not found in $FRONTEND_ENV"
    BETTER_AUTH_SECRET="default-dev-secret-please-change"
fi

if [ -z "$JWT_SECRET" ]; then
    echo "Warning: JWT_SECRET_KEY not found in $BACKEND_ENV"
    JWT_SECRET="default-dev-secret-please-change"
fi

if [ -z "$GEMINI_KEY" ]; then
    echo "Warning: GEMINI_API_KEY not found in $BACKEND_ENV"
    GEMINI_KEY="default-gemini-key-please-change"
fi

echo "✓ Secrets extracted successfully"

# Deploy with Helm
echo ""
echo "Deploying to Kubernetes with Helm..."
helm upgrade --install task-management . \
    --set frontend.image.pullPolicy=Never \
    --set backend.image.pullPolicy=Never \
    --set "frontend.secrets.betterAuthSecret=$BETTER_AUTH_SECRET" \
    --set "backend.secrets.secretKey=$JWT_SECRET" \
    --set "backend.secrets.geminiApiKey=$GEMINI_KEY" \
    --wait \
    --timeout 5m

echo ""
echo "=========================================="
echo "Deployment Complete!"
echo "=========================================="
echo ""
echo "Secrets loaded from .env files:"
echo "  ✓ BETTER_AUTH_SECRET (from frontend-app/.env)"
echo "  ✓ JWT_SECRET_KEY (from backend/.env)"
echo "  ✓ GEMINI_API_KEY (from backend/.env)"
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
echo "To view secrets (base64 encoded):"
echo "  kubectl get secrets"
echo ""
