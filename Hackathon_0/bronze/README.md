# AI Employee - Bronze Tier Implementation

## Overview
This is a Bronze tier implementation of the Personal AI Employee Hackathon. It provides a foundational autonomous task management system using Claude Code, Obsidian, and Python watchers.

## Features Implemented (Bronze Tier)
✅ Obsidian vault with Dashboard.md and Company_Handbook.md
✅ File System Watcher for monitoring drop folders
✅ Claude Code integration via Agent Skills
✅ Basic folder structure: /Inbox, /Needs_Action, /Done, /Plans, /Logs
✅ Human-in-the-Loop approval workflow
✅ All AI functionality implemented as Agent Skills

## Architecture

```
AI_Employee_Vault/
├── vault/                    # Obsidian vault (open this in Obsidian)
│   ├── Dashboard.md          # Real-time system status
│   ├── Company_Handbook.md   # Rules and guidelines
│   ├── Inbox/                # New items from watchers
│   ├── Needs_Action/         # Tasks requiring processing
│   ├── Done/                 # Completed tasks
│   ├── Plans/                # Task plans and strategies
│   ├── Logs/                 # System logs
│   ├── Pending_Approval/     # Actions awaiting approval
│   ├── Approved/             # Approved actions
│   ├── Rejected/             # Rejected actions
│   └── Briefings/            # Daily/weekly briefings
├── base_watcher.py           # Base watcher class
├── filesystem_watcher.py     # File system monitoring
├── test_runner.py            # Automated testing
├── requirements.txt          # Python dependencies
├── setup.sh                  # Linux/Mac setup
└── setup.bat                 # Windows setup
```

## Prerequisites

### Required Software
- **Claude Code**: Active subscription or API access
- **Obsidian**: v1.10.6+ (free) - [Download](https://obsidian.md/download)
- **Python**: 3.13 or higher - [Download](https://www.python.org/downloads/)
- **Git**: For version control - [Download](https://git-scm.com/)

### Hardware Requirements
- Minimum: 8GB RAM, 4-core CPU, 20GB free disk space
- Recommended: 16GB RAM, 8-core CPU, SSD storage

## Installation

### 1. Clone or Download This Repository
```bash
cd E:\hackathon-0\bronze
```

### 2. Install Python Dependencies
```bash
cd AI_Employee_Vault
pip install -r requirements.txt
```

### 3. Set Up Obsidian
1. Open Obsidian
2. Click "Open folder as vault"
3. Select the `AI_Employee_Vault/vault` folder
4. The vault should now be accessible in Obsidian

### 4. Verify Claude Code Installation
```bash
claude --version
```

If not installed, follow: https://claude.com/product/claude-code

### 5. Create Drop Folder
```bash
mkdir Drop_Folder
```

## Usage

### Starting the File System Watcher

The watcher monitors a drop folder and automatically creates task files when new files are detected.

```bash
cd AI_Employee_Vault
python filesystem_watcher.py
```

Or specify custom paths:
```bash
python filesystem_watcher.py ./AI_Employee_Vault ./Drop_Folder
```

### Using Claude Code with Agent Skills

#### 1. Process Pending Tasks
```bash
cd AI_Employee_Vault
claude
```

Then in Claude Code:
```
/vault-manager
Check the vault and process any pending tasks
```

#### 2. Generate Daily Briefing
```
/daily-briefing
Generate today's daily briefing
```

#### 3. Manual Task Processing
```
Read all files in Needs_Action folder and process them according to Company_Handbook.md rules
```

### Testing the System

#### Test 1: File Drop Detection
1. Start the watcher: `python filesystem_watcher.py`
2. Drop a test file into `Drop_Folder/`
3. Check `Inbox/` - file should be copied there
4. Check `Needs_Action/` - metadata file should be created

#### Test 2: Claude Code Integration
1. Navigate to vault: `cd AI_Employee_Vault`
2. Start Claude Code: `claude`
3. Ask Claude to read Dashboard.md
4. Ask Claude to update Dashboard.md with current timestamp
5. Verify changes in Obsidian

#### Test 3: Task Processing Workflow
1. Create a test task in `Needs_Action/TEST_TASK.md`
2. Use `/vault-manager` skill to process it
3. Verify task moves to `Done/` after completion
4. Check Dashboard.md for updated activity

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
- "Generate daily briefing"
- "Create morning summary"
- "What happened today?"

## Workflow Examples

### Example 1: Processing a Dropped File
1. User drops `invoice.pdf` into Drop_Folder
2. Watcher detects file and copies to Inbox
3. Watcher creates `FILE_invoice.md` in Needs_Action
4. User runs Claude Code with `/vault-manager`
5. Claude reads the task, creates a plan
6. Claude updates Dashboard with activity
7. Task moves to Done when complete

### Example 2: Daily Morning Routine
1. User opens terminal and runs: `claude`
2. User: `/daily-briefing generate today's briefing`
3. Claude analyzes completed and pending tasks
4. Claude creates briefing in Briefings folder
5. Claude updates Dashboard
6. User reviews briefing in Obsidian

## Folder Descriptions

| Folder | Purpose |
|--------|---------|
| **Inbox** | Files copied from drop folder |
| **Needs_Action** | Tasks requiring processing |
| **Done** | Completed tasks (archive after 30 days) |
| **Plans** | Task plans for complex items |
| **Logs** | System logs and audit trails |
| **Pending_Approval** | Actions awaiting human approval |
| **Approved** | Approved actions ready for execution |
| **Rejected** | Rejected actions |
| **Briefings** | Daily/weekly briefings |

## Security & Privacy

### Credentials
- Never commit `.env` files
- Store sensitive data outside the vault
- Use environment variables for API keys

### Audit Logging
- All actions logged to `/Logs`
- Review logs weekly
- Retain logs for 90 days minimum

### Approval Workflow
- Sensitive actions require approval (see Company_Handbook.md)
- Review Pending_Approval folder daily
- Move to Approved or Rejected as appropriate

## Troubleshooting

### Watcher Not Detecting Files
- Verify Drop_Folder path is correct
- Check file permissions
- Ensure watcher is running (check terminal)

### Claude Code Can't Read Vault
- Verify you're in the vault directory
- Check file permissions
- Try absolute paths

### Skills Not Working
- Verify `.claude/skills/` directory exists
- Check SKILL.md files are properly formatted
- Restart Claude Code

## Next Steps (Silver/Gold Tier)

To upgrade to Silver tier, add:
- Gmail watcher with API integration
- WhatsApp watcher (Playwright-based)
- MCP server for sending emails
- Scheduled automation (cron/Task Scheduler)

To upgrade to Gold tier, add:
- Multiple MCP servers
- Weekly business audit
- Odoo accounting integration
- Ralph Wiggum loop for autonomous completion

## Resources

- [Hackathon Document](./Personal%20AI%20Employee%20Hackathon%200_%20Building%20Autonomous%20FTEs%20in%202026.md)
- [Claude Code Documentation](https://agentfactory.panaversity.org/docs/AI-Tool-Landscape/claude-code-features-and-workflows)
- [Agent Skills Guide](https://platform.claude.com/docs/en/agents-and-tools/agent-skills/overview)
- [Obsidian Help](https://help.obsidian.md/)

## Contributing

This is a hackathon project. Feel free to:
- Add more watchers (Gmail, WhatsApp, etc.)
- Create additional Agent Skills
- Improve error handling
- Add more automation

## License

MIT License - Feel free to use and modify

## Support

- Weekly Research Meetings: Wednesdays at 10:00 PM
- Zoom: https://us06web.zoom.us/j/87188707642?pwd=a9XloCsinvn1JzICbPc2YGUvWTbOTr.1
- YouTube: https://www.youtube.com/@panaversity

---

**Built for Personal AI Employee Hackathon 0 - Bronze Tier**
