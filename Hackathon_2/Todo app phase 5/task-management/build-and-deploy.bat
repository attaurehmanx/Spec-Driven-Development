@echo off
REM Build and Deploy Task Management Application to Minikube (Windows)
REM This script builds Docker images and deploys them to minikube

echo ==========================================
echo Task Management - Build and Deploy Script
echo ==========================================

REM Check if minikube is running
minikube status >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Minikube is not running. Please start minikube first.
    echo Run: minikube start
    exit /b 1
)

REM Set Docker environment to use minikube's Docker daemon
echo Setting Docker environment to minikube...
@FOR /f "tokens=*" %%i IN ('minikube -p minikube docker-env --shell cmd') DO @%%i

REM Build Backend Image
echo.
echo Building Backend Docker image...
cd ..\backend
docker build -t task-management-backend:latest .
if %errorlevel% neq 0 (
    echo Error: Failed to build backend image
    exit /b 1
)
echo Backend image built successfully

REM Build Frontend Image
echo.
echo Building Frontend Docker image...
cd ..\frontend-app
docker build -t task-management-frontend:latest .
if %errorlevel% neq 0 (
    echo Error: Failed to build frontend image
    exit /b 1
)
echo Frontend image built successfully

REM Return to helm chart directory
cd ..\task-management

REM Deploy with Helm
echo.
echo Deploying to Kubernetes with Helm...
helm upgrade --install task-management . --set frontend.image.pullPolicy=Never --set backend.image.pullPolicy=Never --wait --timeout 5m

echo.
echo ==========================================
echo Deployment Complete!
echo ==========================================
echo.
echo To access your application:
echo   Frontend: minikube service task-management-frontend
echo   Backend:  minikube service task-management-backend
echo.
echo Or get the URLs:
echo   minikube service list
echo.
echo To check pod status:
echo   kubectl get pods
echo.
