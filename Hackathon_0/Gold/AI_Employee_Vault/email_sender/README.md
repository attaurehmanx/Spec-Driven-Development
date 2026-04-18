# Email Sender - Gold Tier

Gmail API integration for sending emails.

## File

| File | Purpose |
|------|---------|
| `send_email_tasks.py` | Processes email tasks from **Approved** folder and sends via Gmail API |

## Usage

```bash
cd AI_Employee_Vault
python email_sender/send_email_tasks.py "path/to/EMAIL_*.md"
```

## Workflow

```
Gmail Watcher → Needs_Action/email/EMAIL_*.md
                    ↓
            Vault Manager reads task
                    ↓
            Creates plan in Plans/email/
                    ↓
            Shows user proposed response
                    ↓
            User chooses option:
            ┌─────────────────────────┐
            │ a. Send                 │
            │ b. Edit                 │
            │ c. Manual               │
            │ d. Reject               │
            └─────────────────────────┘
                    ↓
            If 'a' (Send):
            Move to Approved/email/
                    ↓
            Run send_email_tasks.py
                    ↓
            Email sent via Gmail API
                    ↓
            Moved to Done/email/
            Logged to Logs/YYYY-MM-DD.json
```

## Setup

Ensure credentials exist in `AI_Employee_Vault/`:
- `credentials.json` - OAuth2 credentials
- `gmail_token.json` - Auth token (auto-generated on first run)

## Dependencies

```bash
pip install google-auth google-auth-oauthlib google-api-python-client
```
