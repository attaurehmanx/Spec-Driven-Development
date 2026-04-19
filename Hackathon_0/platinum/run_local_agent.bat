@echo off
REM run_local_agent.bat
REM Run Local Agent from project root (correct vault location)

echo ============================================
echo Starting Local Agent
echo ============================================
echo.

REM Get script directory (project root)
set SCRIPT_DIR=%~dp0

echo Project Root: %SCRIPT_DIR%
echo Vault Location: %SCRIPT_DIR%vault
echo.

REM Change to project root
cd /d %SCRIPT_DIR%

REM Run Local Agent
python local\agents\local_agent_fixed.py

pause
