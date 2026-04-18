---
name: vault-manager
description: Process tasks from Needs_Action folder, create plans, manage approvals with interactive user confirmation
---

# Vault Manager - Task Processing & Workflow Management

## Workflow

### Step 1: Scan Needs_Action Folder
```bash
cd AI_Employee_Vault/vault/Needs_Action
dir /s /b *.md
```

### Step 2: Read and Analyze Tasks
For each task file:
1. Read metadata (type, priority, from, subject)
2. Check Company_Handbook.md for approval rules
3. Determine required action

### Step 3: Create Plan
Create execution plan in `Plans/<domain>/` folder

### Step 4: Email Workflow (Interactive)

For email tasks:
1. Generate draft response
2. Present user with options:

```
═══════════════════════════════════════════════════════════
📧 Email Task Detected

From: {sender}
Subject: {subject}

📝 Proposed Response:
{draft}

───────────────────────────────────────────────────────────
Options:
  [a] Send the email
  [b] Edit response
  [c] Mark as done (manual reply)
  [d] Reject/Archive
═══════════════════════════════════════════════════════════
```

### Step 5: Execute Based on User Choice

| Choice | Action |
|--------|--------|
| **a** | Move to `Approved/email/` → Run `send_email_tasks.py` |
| **b** | Edit draft, then re-prompt |
| **c** | Move to `Done/email/` (no send) |
| **d** | Move to `Rejected/email/` |

### Step 6: Complete Task
1. Log action to `Logs/YYYY-MM-DD.json`
2. Update `Dashboard.md`
3. Move task to final folder

## Email Response Templates

### Gold Tier Progress
```
Hi,

Thanks for reaching out! Here's the current Gold Tier progress:

**System Status**: Fully Operational
- 8/8 watchers active and running
- Processing tasks autonomously

**Recent Activity**:
- 16+ social media interactions processed
- Instagram comment replies: 15 completed
- Facebook comment replies: 1 completed

**Business Metrics**:
- Revenue: $0 | Expenses: $0 | Uptime: 100%

Weekly CEO briefings scheduled for Sunday 8:00 PM.

Best regards,
AI Employee Assistant
```

### Default Response
```
Hi {name},

Thank you for your email regarding "{subject}".

Your message has been received. We will respond within 24 hours.

Best regards,
AI Employee Assistant
```

## Available Tools

### Send Email
```bash
cd AI_Employee_Vault
python email_sender/send_email_tasks.py "path/to/EMAIL_*.md"
```

### Facebook Comment Reply
```bash
cd AI_Employee_Vault/mcp_servers/social_media
python reply_facebook_comment.py "<comment_id>" "<message>"
```

### Instagram Comment Reply
```python
import subprocess, json
proc = subprocess.Popen(
    ['python', 'AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_comment.py'],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
)
request = {
    'jsonrpc': '2.0', 'id': 1,
    'method': 'tools/call',
    'params': {
        'name': 'social_reply_instagram_comment',
        'arguments': {'comment_id': 'ID', 'message': 'Thanks!'}
    }
}
proc.stdin.write(json.dumps(request) + '\n')
proc.stdin.flush()
proc.stdin.close()
result = json.loads(proc.stdout.read())
```

## Execution Rules

1. **ALWAYS ask for confirmation** before sending emails
2. **Present 4 options**: Send, Edit, Manual, Reject
3. **For 'Send' option**: Move to Approved/email/ first, then run send_email_tasks.py
4. **Log all actions** to daily log file
5. **Update Dashboard** after each task
6. **Escalate errors** after 3 retries

## Task Flow

```
Gmail Watcher
     ↓
Needs_Action/email/EMAIL_*.md
     ↓
Vault Manager reads task
     ↓
Creates plan in Plans/email/
     ↓
Generates draft response
     ↓
┌──────────────────────────────────┐
│  USER CONFIRMATION               │
│  a → Move to Approved/ + send    │
│  b → Edit & re-prompt            │
│  c → Done (manual)               │
│  d → Rejected                    │
└──────────────────────────────────┘
     ↓
Execute action
     ↓
Log → Logs/YYYY-MM-DD.json
     ↓
Update → Dashboard.md
     ↓
Move → Done/Approved/Rejected/
```
