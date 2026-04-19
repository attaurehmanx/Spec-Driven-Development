@echo off
REM authenticate_gmail.bat
REM One-time Gmail OAuth authentication

echo ============================================
echo Gmail OAuth Authentication
echo ============================================
echo.
echo This will authenticate with Gmail API for both:
echo - Cloud Agent (read emails)
echo - Local Agent (send emails)
echo.
echo A browser window will open.
echo Sign in and click "Allow" to grant permissions.
echo.
pause

cd /d %~dp0

python -c "
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from pathlib import Path
import pickle

# Scopes for both reading and sending emails
SCOPES = [
    'https://www.googleapis.com/auth/gmail.readonly',
    'https://www.googleapis.com/auth/gmail.send'
]

# Paths
credentials_path = Path('credentials.json')
token_path = Path('gmail_token.json')

if not credentials_path.exists():
    print('ERROR: credentials.json not found!')
    print('Copy it from Gold tier or download from Google Cloud Console')
    exit(1)

print('Starting Gmail OAuth flow...')
print(f'Credentials: {credentials_path}')
print(f'Token will be saved to: {token_path}')
print()

# Start OAuth flow
flow = InstalledAppFlow.from_client_secrets_file(
    str(credentials_path),
    SCOPES
)

# Run local server for callback
creds = flow.run_local_server(port=0)

# Save token
with open(token_path, 'wb') as token:
    pickle.dump(creds, token)

print()
print('============================================')
print('SUCCESS! Gmail authentication complete!')
print('============================================')
print()
print('Token saved to: gmail_token.json')
print()
print('You can now run:')
print('  - Cloud Agent: python cloud\\agents\\cloud_agent_fixed.py')
print('  - Local Agent: python local\\agents\\local_agent_fixed.py')
print()
"

if errorlevel 1 (
    echo.
    echo Authentication FAILED!
    echo Check the error message above.
    pause
    exit /b 1
)

echo.
echo Press any key to exit...
pause >nul
