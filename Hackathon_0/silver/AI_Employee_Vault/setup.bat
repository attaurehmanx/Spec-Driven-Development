@echo off
REM AI Employee Setup Script - Windows (Silver Tier)
REM Run this script to set up your AI Employee system

echo ============================================================
echo AI Employee - Silver Tier Setup (Windows)
echo ============================================================
echo.

REM Check Python version
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.13 or higher from python.org
    pause
    exit /b 1
)

echo [1/6] Checking Python version...
python -c "import sys; exit(0 if sys.version_info >= (3, 13) else 1)"
if errorlevel 1 (
    echo ERROR: Python 3.13 or higher is required
    pause
    exit /b 1
)
echo ✓ Python version OK

echo.
echo [2/6] Installing Python dependencies...
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    pause
    exit /b 1
)
echo ✓ Dependencies installed

echo.
echo [3/6] Creating vault directory structure...
if not exist "vault" mkdir vault
if not exist "vault\Inbox" mkdir vault\Inbox
if not exist "vault\Needs_Action" mkdir vault\Needs_Action
if not exist "vault\Done" mkdir vault\Done
if not exist "vault\Plans" mkdir vault\Plans
if not exist "vault\Logs" mkdir vault\Logs
if not exist "vault\Pending_Approval" mkdir vault\Pending_Approval
if not exist "vault\Approved" mkdir vault\Approved
if not exist "vault\Rejected" mkdir vault\Rejected
if not exist "vault\Briefings" mkdir vault\Briefings
if not exist "vault\LinkedIn_Drafts" mkdir vault\LinkedIn_Drafts
echo ✓ Vault structure created

echo.
echo [4/6] Creating Drop Folder...
cd ..
if not exist "Drop_Folder" mkdir Drop_Folder
cd AI_Employee_Vault
echo ✓ Drop Folder created

echo.
echo [5/6] Setting up Playwright (for WhatsApp watcher)...
echo This may take a few minutes...
python -m playwright install chromium
if errorlevel 1 (
    echo ⚠ Playwright installation failed (optional)
    echo WhatsApp watcher will not work without it
) else (
    echo ✓ Playwright installed
)

echo.
echo [6/6] Creating .gitignore...
(
echo # Python
echo __pycache__/
echo *.py[cod]
echo *$py.class
echo *.so
echo .Python
echo venv/
echo env/
echo.
echo # Credentials
echo credentials.json
echo token.json
echo .env
echo.
echo # Sessions
echo whatsapp_session/
echo.
echo # Logs
echo *.log
echo.
echo # OS
echo .DS_Store
echo Thumbs.db
) > .gitignore
echo ✓ .gitignore created

echo.
echo ============================================================
echo Setup Complete!
echo ============================================================
echo.
echo Next Steps:
echo 1. Open Obsidian and open the 'vault' folder as a vault
echo 2. Review vault/Company_Handbook.md
echo 3. Start the file system watcher:
echo    python filesystem_watcher.py
echo.
echo Optional Setup:
echo - Gmail: Place credentials.json in this directory
echo - WhatsApp: Run whatsapp_watcher.py and scan QR code
echo.
echo To start all watchers:
echo    python orchestrator.py
echo.
echo ============================================================
pause
