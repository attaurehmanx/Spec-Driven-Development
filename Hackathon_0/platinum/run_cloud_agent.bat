@echo off
REM run_cloud_agent.bat
REM Run Cloud Agent from project root (correct vault location)

echo ============================================
echo Starting Cloud Agent
echo ============================================
echo.

REM Get script directory (project root)
set SCRIPT_DIR=%~dp0

echo Project Root: %SCRIPT_DIR%
echo Vault Location: %SCRIPT_DIR%vault
echo.

REM Change to project root
cd /d %SCRIPT_DIR%

REM Run Cloud Agent
python cloud\agents\cloud_agent_fixed.py

pause
