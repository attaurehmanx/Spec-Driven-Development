# Automated deployment script with secrets from .env files (PowerShell)
# This script extracts secrets from your .env files and deploys to minikube

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "Task Management - Automated Deployment" -ForegroundColor Cyan
Write-Host "with Secrets from .env Files" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan

# Check if minikube is running
try {
    $minikubeStatus = minikube status 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error: Minikube is not running. Please start minikube first." -ForegroundColor Red
        Write-Host "Run: minikube start" -ForegroundColor Yellow
        exit 1
    }
} catch {
    Write-Host "Error: Minikube is not running or not installed." -ForegroundColor Red
    exit 1
}

# Set Docker environment to use minikube's Docker daemon
Write-Host "`nSetting Docker environment to minikube..." -ForegroundColor Yellow
& minikube -p minikube docker-env --shell powershell | Invoke-Expression

# Build Backend Image
Write-Host "`nBuilding Backend Docker image..." -ForegroundColor Yellow
Push-Location ..\backend
docker build -t task-management-backend:latest .
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Failed to build backend image" -ForegroundColor Red
    Pop-Location
    exit 1
}
Write-Host "✓ Backend image built successfully" -ForegroundColor Green
Pop-Location

# Build Frontend Image
Write-Host "`nBuilding Frontend Docker image..." -ForegroundColor Yellow
Push-Location ..\frontend-app
docker build -t task-management-frontend:latest .
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Failed to build frontend image" -ForegroundColor Red
    Pop-Location
    exit 1
}
Write-Host "✓ Frontend image built successfully" -ForegroundColor Green
Pop-Location

# Extract secrets from .env files
Write-Host "`nExtracting secrets from .env files..." -ForegroundColor Yellow

$frontendEnvPath = "..\frontend-app\.env"
$backendEnvPath = "..\backend\.env"

if (-not (Test-Path $frontendEnvPath)) {
    Write-Host "Error: Frontend .env file not found at $frontendEnvPath" -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $backendEnvPath)) {
    Write-Host "Error: Backend .env file not found at $backendEnvPath" -ForegroundColor Red
    exit 1
}

# Read .env files
$frontendEnv = Get-Content $frontendEnvPath
$backendEnv = Get-Content $backendEnvPath

# Extract secrets
$betterAuthSecret = ($frontendEnv | Select-String "^BETTER_AUTH_SECRET=").ToString() -replace '^BETTER_AUTH_SECRET=', '' -replace '"', '' -replace "'", ''
$jwtSecret = ($backendEnv | Select-String "^SECRET_KEY=").ToString() -replace '^SECRET_KEY=', '' -replace '"', '' -replace "'", ''
$geminiKey = ($backendEnv | Select-String "^GEMINI_API_KEY=").ToString() -replace '^GEMINI_API_KEY=', '' -replace '"', '' -replace "'", ''

# Validate secrets were extracted
if ([string]::IsNullOrWhiteSpace($betterAuthSecret)) {
    Write-Host "Warning: BETTER_AUTH_SECRET not found in $frontendEnvPath" -ForegroundColor Yellow
    $betterAuthSecret = "default-dev-secret-please-change"
}

if ([string]::IsNullOrWhiteSpace($jwtSecret)) {
    Write-Host "Warning: SECRET_KEY not found in $backendEnvPath" -ForegroundColor Yellow
    $jwtSecret = "default-dev-secret-please-change"
}

if ([string]::IsNullOrWhiteSpace($geminiKey)) {
    Write-Host "Warning: GEMINI_API_KEY not found in $backendEnvPath" -ForegroundColor Yellow
    $geminiKey = "default-gemini-key-please-change"
}

Write-Host "✓ Secrets extracted successfully" -ForegroundColor Green

# Deploy with Helm
Write-Host "`nDeploying to Kubernetes with Helm..." -ForegroundColor Yellow
helm upgrade --install task-management . `
    --set frontend.image.pullPolicy=Never `
    --set backend.image.pullPolicy=Never `
    --set "frontend.secrets.betterAuthSecret=$betterAuthSecret" `
    --set "backend.secrets.secretKey=$jwtSecret" `
    --set "backend.secrets.geminiApiKey=$geminiKey" `
    --wait `
    --timeout 5m

if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Helm deployment failed" -ForegroundColor Red
    exit 1
}

Write-Host "`n==========================================" -ForegroundColor Cyan
Write-Host "Deployment Complete!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Secrets loaded from .env files:" -ForegroundColor Green
Write-Host "  ✓ BETTER_AUTH_SECRET (from frontend-app\.env)"
Write-Host "  ✓ SECRET_KEY (from backend\.env)"
Write-Host "  ✓ GEMINI_API_KEY (from backend\.env)"
Write-Host ""
Write-Host "To access your application:" -ForegroundColor Yellow
Write-Host "  Frontend: minikube service task-management-frontend"
Write-Host "  Backend:  minikube service task-management-backend"
Write-Host ""
Write-Host "Or get the URLs:" -ForegroundColor Yellow
Write-Host "  minikube service list"
Write-Host ""
Write-Host "To check pod status:" -ForegroundColor Yellow
Write-Host "  kubectl get pods"
Write-Host ""
Write-Host "To view secrets (base64 encoded):" -ForegroundColor Yellow
Write-Host "  kubectl get secrets"
Write-Host ""
