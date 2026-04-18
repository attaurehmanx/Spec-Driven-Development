# AI Employee - Silver Tier Implementation

**Status**: ✅ Complete
**Tier**: Silver (Functional Assistant)
**Estimated Setup Time**: 20-30 hours

---

## Overview

This is a Silver tier implementation of the Personal AI Employee Hackathon. It provides an autonomous task management system with multiple watchers, MCP integration, and human-in-the-loop approval workflows.

## Features Implemented (Silver Tier)

✅ All Bronze tier requirements
✅ Multiple Watcher scripts (File System, Gmail, WhatsApp, LinkedIn)
✅ Automated LinkedIn posting workflow
✅ Claude Code reasoning loop with Plan.md creation
✅ MCP server integration ready (email sending)
✅ Human-in-the-loop approval workflow
✅ Scheduled automation support (cron/Task Scheduler)
✅ All AI functionality as Agent Skills

---

## Architecture

```
AI_Employee_Vault/
├── vault/                          # Obsidian vault (open in Obsidian)
│   ├── Dashboard.md                # Real-time system status
│   ├── Company_Handbook.md         # Rules and guidelines
│   │
│   ├── Inbox/                      # New items from watchers
│   │   ├── filesystem/             # Files from Drop_Folder
│   │   ├── gmail/                  # Email attachments/content
│   │   ├── linkedin/               # LinkedIn-related items
│   │   └── whatsapp/               # WhatsApp attachments
│   │
│   ├── Needs_Action/               # Tasks requiring processing
│   │   ├── filesystem/             # File processing tasks
│   │   ├── gmail/                  # Email response tasks
│   │   ├── linkedin/               # LinkedIn posting tasks
│   │   └── whatsapp/               # WhatsApp reply tasks
│   │
│   ├── Plans/                      # Task plans and strategies
│   │   ├── filesystem/             # Plans for file tasks
│   │   ├── gmail/                  # Plans for email tasks
│   │   ├── linkedin/               # Plans for LinkedIn tasks
│   │   └── whatsapp/               # Plans for WhatsApp tasks
│   │
│   ├── Pending_Approval/           # Actions awaiting approval
│   │   ├── filesystem/             # File actions to approve
│   │   ├── gmail/                  # Email drafts to approve
│   │   ├── linkedin/               # LinkedIn posts to approve
│   │   └── whatsapp/               # WhatsApp replies to approve
│   │
│   ├── Approved/                   # Approved actions
│   │   ├── filesystem/             # Approved file actions
│   │   ├── gmail/                  # Approved emails
│   │   ├── linkedin/               # Approved LinkedIn posts
│   │   └── whatsapp/               # Approved WhatsApp replies
│   │
│   ├── Rejected/                   # Rejected actions
│   │   ├── filesystem/             # Rejected file actions
│   │   ├── gmail/                  # Rejected email drafts
│   │   ├── linkedin/               # Rejected LinkedIn posts
│   │   └── whatsapp/               # Rejected WhatsApp replies
│   │
│   ├── Done/                       # Completed tasks
│   │   ├── filesystem/             # Completed file tasks
│   │   ├── gmail/                  # Completed email tasks
│   │   ├── linkedin/               # Completed LinkedIn tasks
│   │   └── whatsapp/               # Completed WhatsApp tasks
│   │
│   ├── Logs/                       # System logs (JSON format)
│   │   ├── filesystem/             # File watcher logs
│   │   ├── gmail/                  # Gmail watcher logs
│   │   ├── linkedin/               # LinkedIn watcher logs
│   │   └── whatsapp/               # WhatsApp watcher logs
│   │
│   ├── Briefings/                  # Daily/weekly briefings
│   └── LinkedIn_Drafts/            # LinkedIn post drafts
│
├── Watchers (Python scripts)
│   ├── base_watcher.py             # Base watcher class
│   ├── filesystem_watcher.py       # File system monitoring
│   ├── gmail_watcher.py            # Gmail monitoring
│   ├── whatsapp_watcher.py         # WhatsApp monitoring
│   ├── linkedin_watcher.py         # LinkedIn posting scheduler
│   └── orchestrator.py             # Process management
│
├── Configuration
│   ├── requirements.txt            # Python dependencies
│   ├── setup.sh                    # Linux/Mac setup
│   ├── setup.bat                   # Windows setup
│   └── orchestrator_config.json    # Watcher configuration
│
└── Drop_Folder/                    # Drop files here to process
```

### Organized Folder Structure

**NEW**: All workflow folders now have source-specific subfolders (filesystem, gmail, linkedin, whatsapp) for better organization and tracking. See [FOLDER_STRUCTURE.md](FOLDER_STRUCTURE.md) for complete documentation.

**Benefits**:
- Clear source tracking for all items
- Better organization and navigation
- Improved analytics by source
- Scalable architecture for new watchers

---

## Prerequisites

### Required Software
- **Claude Code**: Active subscription - [Get Claude Code](https://claude.com/product/claude-code)
- **Obsidian**: v1.10.6+ (free) - [Download](https://obsidian.md/download)
- **Python**: 3.13 or higher - [Download](https://www.python.org/downloads/)
- **Git**: For version control - [Download](https://git-scm.com/)

### Optional (for full functionality)
- **Gmail API Credentials**: For email monitoring
- **Playwright**: For WhatsApp monitoring
- **Node.js**: For MCP servers

### Hardware Requirements
- Minimum: 8GB RAM, 4-core CPU, 20GB free disk space
- Recommended: 16GB RAM, 8-core CPU, SSD storage

---

## Quick Start

### 1. Run Setup Script

**Windows**:
```bash
cd AI_Employee_Vault
setup.bat
```

**Linux/Mac**:
```bash
cd AI_Employee_Vault
chmod +x setup.sh
./setup.sh
```

### 2. Open Vault in Obsidian

1. Open Obsidian
2. Click "Open folder as vault"
3. Select `AI_Employee_Vault/vault` folder
4. Review Dashboard.md and Company_Handbook.md

### 3. Start Watchers

**Option A: Start All Watchers (Orchestrator)**
```bash
cd AI_Employee_Vault
python orchestrator.py
```

**Option B: Start Individual Watchers**
```bash
# File System Watcher
python filesystem_watcher.py

# LinkedIn Watcher (in separate terminal)
python linkedin_watcher.py

# Gmail Watcher (requires credentials.json)
python gmail_watcher.py

# WhatsApp Watcher (requires Playwright)
python whatsapp_watcher.py
```

### 4. Test the System

1. Drop a test file into `Drop_Folder/`
2. Watch the watcher terminal for detection
3. Check `vault/Inbox/` for the copied file
4. Check `vault/Needs_Action/` for the task file

### 5. Process Tasks with Claude Code

```bash
cd AI_Employee_Vault
claude
```

Then use Agent Skills:
```
/vault-manager
Check the vault and process any pending tasks
```

---

## Agent Skills

### vault-manager
Manages the vault, processes tasks, updates dashboard.

**Usage**: `/vault-manager` then describe what you need

**Examples**:
- "Check the vault and process pending tasks"
- "Update the dashboard"
- "Review all items in Needs_Action"

### daily-briefing
Generates daily summaries and briefings.

**Usage**: `/daily-briefing` then request briefing

**Examples**:
- "Generate today's daily briefing"
- "Create weekly summary"

### linkedin-poster
Creates and manages LinkedIn posts.

**Usage**: `/linkedin-poster` then describe what you need

**Examples**:
- "Create a LinkedIn post about [topic]"
- "Draft today's post"

### email-sender
Manages email drafting and sending.

**Usage**: `/email-sender` then describe what you need

**Examples**:
- "Draft a reply to [email]"
- "Send email to [recipient] about [topic]"

---

## Watcher Details

### File System Watcher
- **Purpose**: Monitors Drop_Folder for new files
- **Interval**: Real-time (watchdog)
- **Actions**: Copies to Inbox, creates task in Needs_Action
- **Status**: ✅ Fully functional

### LinkedIn Watcher
- **Purpose**: Creates daily LinkedIn posting tasks
- **Interval**: Every 24 hours
- **Actions**: Creates post template in Needs_Action
- **Status**: ✅ Fully functional

### Gmail Watcher
- **Purpose**: Monitors Gmail for important unread emails
- **Interval**: Every 2 minutes
- **Actions**: Creates email task in Needs_Action
- **Status**: ⚠️ Requires credentials.json
- **Setup**: See [Gmail API Setup](#gmail-api-setup)

### WhatsApp Watcher
- **Purpose**: Monitors WhatsApp Web for urgent messages
- **Interval**: Every 30 seconds
- **Actions**: Creates message task for urgent keywords
- **Status**: ⚠️ Requires Playwright
- **Setup**: See [WhatsApp Setup](#whatsapp-setup)

---

## Optional Setup

### Gmail API Setup

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project
3. Enable Gmail API
4. Create OAuth 2.0 credentials
5. Download credentials as `credentials.json`
6. Place in `AI_Employee_Vault/` directory
7. Run `python gmail_watcher.py` and authorize

### WhatsApp Setup

1. Install Playwright: `pip install playwright`
2. Install browser: `playwright install chromium`
3. Run `python whatsapp_watcher.py`
4. Scan QR code with WhatsApp mobile app
5. Session will be saved for future use

### MCP Server Setup

For email sending functionality:

1. Install email MCP server (Node.js required)
2. Configure in Claude Code settings
3. See [MCP Documentation](https://modelcontextprotocol.io/)

---

## Workflow Examples

### Example 1: Processing a Dropped File
1. User drops `invoice.pdf` into Drop_Folder
2. File System Watcher detects and copies to Inbox
3. Watcher creates `FILE_invoice.md` in Needs_Action
4. User runs `/vault-manager` in Claude Code
5. Claude reads task, creates plan, updates Dashboard
6. Task moves to Done when complete

### Example 2: Daily LinkedIn Post
1. LinkedIn Watcher creates posting task (every 24 hours)
2. Task appears in Needs_Action
3. User runs `/linkedin-poster` in Claude Code
4. Claude drafts post based on business goals
5. Draft moves to Pending_Approval
6. User reviews, approves, posts manually
7. Task moves to Done

### Example 3: Email Processing
1. Gmail Watcher detects important unread email
2. Creates email task in Needs_Action
3. User runs `/email-sender` in Claude Code
4. Claude drafts reply
5. Draft moves to Pending_Approval
6. User reviews and approves
7. MCP server sends email
8. Task moves to Done

---

## Scheduling Automation

### Linux/Mac (cron)

Edit crontab: `crontab -e`

```bash
# Daily briefing at 8 AM
0 8 * * * cd /path/to/AI_Employee_Vault && claude "/daily-briefing generate today's briefing"

# Process tasks every hour
0 * * * * cd /path/to/AI_Employee_Vault && claude "/vault-manager process pending tasks"
```

### Windows (Task Scheduler)

1. Open Task Scheduler
2. Create Basic Task
3. Set trigger (daily, hourly, etc.)
4. Action: Start a program
   - Program: `claude`
   - Arguments: `"/daily-briefing generate today's briefing"`
   - Start in: `C:\path\to\AI_Employee_Vault`

---

## Security & Privacy

### Credentials Management
- Never commit credentials to git
- Use environment variables for API keys
- Store sensitive data outside vault
- Rotate credentials monthly

### Audit Logging
- All actions logged to `vault/Logs/`
- JSON format with timestamps
- Review logs weekly
- Retain for 90 days minimum

### Approval Workflow
- Sensitive actions require approval
- Review `Pending_Approval/` daily
- Move to `Approved/` or `Rejected/`
- All decisions logged

---

## Troubleshooting

### Watchers Not Starting
- Check Python version: `python --version`
- Install dependencies: `pip install -r requirements.txt`
- Check file permissions
- Review watcher.log for errors

### Claude Code Can't Read Vault
- Verify working directory: `pwd`
- Use absolute paths if needed
- Check file permissions
- Restart Claude Code

### Gmail Watcher Issues
- Verify credentials.json exists
- Check Gmail API is enabled
- Re-authorize if token expired
- Check internet connection

### WhatsApp Watcher Issues
- Install Playwright: `playwright install chromium`
- Scan QR code on first run
- Check session folder permissions
- WhatsApp Web must be accessible

---

## Upgrade Path

### To Gold Tier (40+ hours)
- [ ] Multiple MCP servers
- [ ] Weekly business audit
- [ ] Odoo accounting integration
- [ ] Ralph Wiggum autonomous loop
- [ ] Comprehensive error recovery
- [ ] Cross-domain integration

### To Platinum Tier (60+ hours)
- [ ] Cloud deployment (24/7)
- [ ] Work-zone specialization
- [ ] Delegation via synced vault
- [ ] Advanced security rules
- [ ] Production monitoring

---

## Testing

Run the test suite:
```bash
python test_runner.py
```

Expected output: All tests passing

---

## Support & Resources

**Weekly Research Meetings**: Wednesdays at 10:00 PM
**Zoom**: https://us06web.zoom.us/j/87188707642?pwd=a9XloCsinvn1JzICbPc2YGUvWTbOTr.1
**YouTube**: https://www.youtube.com/@panaversity
**Submission Form**: https://forms.gle/JR9T1SJq5rmQyGkGA

**Documentation**:
- [Hackathon Guide](../Personal%20AI%20Employee%20Hackathon%200_%20Building%20Autonomous%20FTEs%20in%202026.md)
- [Claude Code Docs](https://agentfactory.panaversity.org/docs/AI-Tool-Landscape/claude-code-features-and-workflows)
- [Agent Skills Guide](https://platform.claude.com/docs/en/agents-and-tools/agent-skills/overview)

---

## Contributing

Improvements welcome:
- Add more watchers
- Create additional Agent Skills
- Improve error handling
- Add more automation
- Enhance documentation

---

## License

MIT License - Feel free to use and modify

---

**Built for Personal AI Employee Hackathon 0 - Silver Tier**
**Completion Date**: 2026-02-20
