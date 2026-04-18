# Email Sender - Gold Tier

Simple Gmail API integration (Silver Tier model).

## Setup

### Install Dependencies

```bash
pip install google-auth google-auth-oauthlib google-api-python-client
```

### Configure Credentials

Credentials should already exist in `AI_Employee_Vault/`:
- `credentials.json` - OAuth2 credentials from Google Cloud
- `token.json` - Auth token (auto-generated on first run)
- `token_sender.json` - Send-specific token (auto-generated)

## Usage

### Send All Approved Emails

```bash
cd AI_Employee_Vault
python send_approved_email.py
```

### Watch Mode (Continuous)

```bash
python send_approved_email.py . --quiet watch
```

## Workflow

1. Email tasks created in `Needs_Action/email/`
2. AI creates draft response in `Pending_Approval/email/` or `Approved/email/`
3. Human moves to `Approved/email/` (or auto-approved per handbook)
4. This script sends emails from `Approved/email/`
5. Completed emails moved to `Done/email/`
6. All actions logged to `Logs/YYYY-MM-DD.json`

## Folder Structure

```
AI_Employee_Vault/
├── credentials.json          # Gmail OAuth credentials
├── token.json                # Auth token (auto-generated)
├── token_sender.json         # Send token (auto-generated)
└── vault/
    ├── Needs_Action/email/   # Incoming email tasks
    ├── Approved/email/       # Approved emails ready to send
    ├── Done/email/           # Completed emails
    └── Logs/                 # Action logs
```

## How It Works

The script:
1. Scans `Approved/email/` and subfolders
2. Parses approval files (`APPROVAL_*.md`)
3. Extracts: to, subject, body
4. Sends via Gmail API
5. Moves to `Done/email/`
6. Logs action

## Example Approval File

```markdown
---
type: approval_request
domain: email
---

# Approval Request

**To**: client@example.com
**Subject**: Re: Invoice #123

### Email Body

Hi Client,

Please find attached...

---
```

## Testing

```bash
# Check credentials
python -c "from google.oauth2.credentials import Credentials; print('OK')"

# Run one-time send
python send_approved_email.py

# Run in watch mode
python send_approved_email.py . --quiet watch
```
