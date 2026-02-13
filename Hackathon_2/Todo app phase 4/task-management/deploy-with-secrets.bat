@echo off
REM Automated deployment script with secrets from .env files (Windows)
REM This script extracts secrets from your .env files and deploys to minikube

echo ==========================================
echo Task Management - Automated Deployment
echo with Secrets from .env Files
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

REM Extract secrets from .env files
echo.
echo Extracting secrets from .env files...

set FRONTEND_ENV=..\frontend-app\.env
set BACKEND_ENV=..\backend\.env

if not exist "%FRONTEND_ENV%" (
    echo Error: Frontend .env file not found at %FRONTEND_ENV%
    exit /b 1
)

if not exist "%BACKEND_ENV%" (
    echo Error: Backend .env file not found at %BACKEND_ENV%
    exit /b 1
)

REM Extract BETTER_AUTH_SECRET
for /f "tokens=2 delims==" %%a in ('findstr /r "^BETTER_AUTH_SECRET=" "%FRONTEND_ENV%"') do set BETTER_AUTH_SECRET=%%a

REM Extract JWT_SECRET_KEY
for /f "tokens=2 delims==" %%a in ('findstr /r "^JWT_SECRET_KEY=" "%BACKEND_ENV%"') do set JWT_SECRET=%%a

REM Extract GEMINI_API_KEY
for /f "tokens=2 delims==" %%a in ('findstr /r "^GEMINI_API_KEY=" "%BACKEND_ENV%"') do set GEMINI_KEY=%%a

REM Validate secrets were extracted
if "%BETTER_AUTH_SECRET%"=="" (
    echo Warning: BETTER_AUTH_SECRET not found in %FRONTEND_ENV%
    set BETTER_AUTH_SECRET=default-dev-secret-please-change
)

if "%JWT_SECRET%"=="" (
    echo Warning: JWT_SECRET_KEY not found in %BACKEND_ENV%
    set JWT_SECRET=default-dev-secret-please-change
)

if "%GEMINI_KEY%"=="" (
    echo Warning: GEMINI_API_KEY not found in %BACKEND_ENV%
    set GEMINI_KEY=default-gemini-key-please-change
)

echo Secrets extracted successfully

REM Deploy with Helm
echo.
echo Deploying to Kubernetes with Helm...
helm upgrade --install task-management . --set frontend.image.pullPolicy=Never --set backend.image.pullPolicy=Never --set "frontend.secrets.betterAuthSecret=%BETTER_AUTH_SECRET%" --set "backend.secrets.secretKey=%JWT_SECRET%" --set "backend.secrets.geminiApiKey=%GEMINI_KEY%" --wait --timeout 5m

echo.
echo ==========================================
echo Deployment Complete!
echo ==========================================
echo.
echo Secrets loaded from .env files:
echo   - BETTER_AUTH_SECRET (from frontend-app\.env)
echo   - JWT_SECRET_KEY (from backend\.env)
echo   - GEMINI_API_KEY (from backend\.env)
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
echo To view secrets (base64 encoded):
echo   kubectl get secrets
echo.
