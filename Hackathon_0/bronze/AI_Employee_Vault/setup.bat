@echo off
REM AI Employee - Quick Start Script for Windows
REM This script helps you get started with the AI Employee system

echo ==================================
echo AI Employee - Bronze Tier Setup
echo ==================================
echo.

REM Check if we're in the right directory
if not exist "vault" (
    echo Error: Please run this script from the AI_Employee_Vault directory
    exit /b 1
)

REM Create Drop_Folder if it doesn't exist
if not exist "..\Drop_Folder" (
    echo Creating Drop_Folder...
    mkdir "..\Drop_Folder"
    echo [OK] Drop_Folder created
) else (
    echo [OK] Drop_Folder already exists
)

REM Check Python installation
echo.
echo Checking Python installation...
python --version >nul 2>&1
if %errorlevel% equ 0 (
    for /f "tokens=*" %%i in ('python --version') do set PYTHON_VERSION=%%i
    echo [OK] !PYTHON_VERSION! found
) else (
    echo [ERROR] Python not found. Please install Python 3.13 or higher
    exit /b 1
)

REM Check if requirements are installed
echo.
echo Checking Python dependencies...
python -c "import watchdog" >nul 2>&1
if %errorlevel% equ 0 (
    echo [OK] Dependencies already installed
) else (
    echo Installing dependencies...
    pip install -r requirements.txt
    if %errorlevel% equ 0 (
        echo [OK] Dependencies installed successfully
    ) else (
        echo [ERROR] Failed to install dependencies
        exit /b 1
    )
)

REM Check Claude Code installation
echo.
echo Checking Claude Code installation...
claude --version >nul 2>&1
if %errorlevel% equ 0 (
    echo [OK] Claude Code is installed
) else (
    echo [WARNING] Claude Code not found. Please install from: https://claude.com/product/claude-code
)

echo.
echo ==================================
echo Setup Complete!
echo ==================================
echo.
echo Next Steps:
echo.
echo 1. Start the File System Watcher:
echo    python filesystem_watcher.py
echo.
echo 2. In another terminal, start Claude Code:
echo    cd %CD%
echo    claude
echo.
echo 3. Test the system:
echo    - Drop a file into Drop_Folder
echo    - Use /vault-manager to process tasks
echo    - Use /daily-briefing to generate reports
echo.
echo 4. Open Obsidian:
echo    - Open this folder as a vault
echo    - View Dashboard.md for system status
echo.
echo ==================================
pause
