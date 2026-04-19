@echo off
REM copy_credentials.bat
REM Copy credentials from Gold to Platinum tier

echo ============================================
echo Copying Credentials from Gold to Platinum
echo ============================================
echo.

REM Check if Gold credentials exist
if not exist "E:\hackathon-0\Gold\AI_Employee_Vault\credentials.json" (
    echo [ERROR] Gold credentials not found!
    echo Please check: E:\hackathon-0\Gold\AI_Employee_Vault\credentials.json
    pause
    exit /b 1
)

REM Check if Gold .env exists
if not exist "E:\hackathon-0\Gold\.env" (
    echo [ERROR] Gold .env file not found!
    echo Please check: E:\hackathon-0\Gold\.env
    pause
    exit /b 1
)

echo [1/3] Copying credentials.json...
copy "E:\hackathon-0\Gold\AI_Employee_Vault\credentials.json" "E:\hackathon-0\platinum\credentials.json"
if %errorlevel% equ 0 (
    echo [OK] credentials.json copied successfully
) else (
    echo [FAILED] Could not copy credentials.json
)

echo.
echo [2/3] Copying .env file...
copy "E:\hackathon-0\Gold\.env" "E:\hackathon-0\platinum\.env"
if %errorlevel% equ 0 (
    echo [OK] .env copied successfully
) else (
    echo [FAILED] Could not copy .env
)

echo.
echo [3/3] Creating .gitignore entry for credentials...
echo.
REM Add credentials to .gitignore if not already there
findstr /C:"credentials.json" "E:\hackathon-0\platinum\.gitignore" >nul
if errorlevel 1 (
    echo credentials.json >> "E:\hackathon-0\platinum\.gitignore"
    echo [OK] Added credentials.json to .gitignore
) else (
    echo [SKIP] credentials.json already in .gitignore
)

echo.
echo ============================================
echo Copy Complete!
echo ============================================
echo.
echo Next steps:
echo 1. Review E:\hackathon-0\platinum\.env
echo 2. Run: pip install -r requirements.txt
echo 3. Run: python local/agents/local_agent_fixed.py
echo    (First run will open browser for Gmail OAuth)
echo.
echo SECURITY REMINDER:
echo - NEVER commit .env or credentials.json to git
echo - Keep these files secure
echo - Rotate credentials monthly
echo.
pause
