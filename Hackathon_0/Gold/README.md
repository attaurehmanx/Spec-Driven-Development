# AI Employee - Gold Tier
## Personal AI Employee Hackathon 0: Building Autonomous FTEs

**Tier**: Gold (Autonomous Employee)
**Status**: Complete
**Version**: 1.0.0
**Last Updated**: 2026-02-28

---

## Overview

This is a Gold Tier implementation of a Personal AI Employee - an autonomous system that manages personal and business affairs 24/7 using Claude Code as the reasoning engine and Obsidian as the management dashboard.

### What This System Does

- **Monitors 8 data sources**: Gmail, WhatsApp, LinkedIn, Facebook, Instagram, Twitter/X, Odoo Accounting, File System
- **Processes tasks autonomously**: Using Ralph Wiggum loops for multi-step workflows
- **Manages social media**: Posts across all platforms with approval workflows
- **Tracks finances**: Integrates with Odoo for accounting and business intelligence
- **Generates insights**: Weekly CEO briefings with actionable recommendations
- **Operates 24/7**: Continuous monitoring with error recovery and graceful degradation

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    EXTERNAL SOURCES                         │
│  Gmail │ WhatsApp │ LinkedIn │ Facebook │ Instagram │       │
│  Twitter │ Odoo Accounting │ File System                    │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                  PERCEPTION LAYER (Watchers)                │
│  8 Python scripts monitoring sources every 30s-10min        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              OBSIDIAN VAULT (Local Storage)                 │
│  /Needs_Action │ /Plans │ /Pending_Approval │ /Done         │
│  Dashboard.md │ Company_Handbook.md │ Business_Goals.md     │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              REASONING LAYER (Claude Code)                  │
│  Read → Analyze → Plan → Request Approval → Execute        │
│  Agent Skills: vault-manager, ceo-briefing, social-poster  │
└────────────────────────┬────────────────────────────────────┘
                         │
              ┌──────────┴──────────┐
              ▼                     ▼
┌──────────────────────┐  ┌──────────────────────────────────┐
│  HUMAN-IN-THE-LOOP   │  │      ACTION LAYER (MCP)          │
│  Review & Approve    │──│  Email │ Social │ Odoo │ Browser │
└──────────────────────┘  └──────────────────────────────────┘
```

---

## Features

### ✅ Gold Tier Requirements

1. **Full Cross-Domain Integration**
   - Personal communications (Gmail, WhatsApp)
   - Business operations (Odoo accounting)
   - Social media (LinkedIn, Facebook, Instagram, Twitter/X)
   - File management (local file system)

2. **Odoo Accounting Integration**
   - JSON-RPC API integration
   - Transaction monitoring
   - Invoice tracking
   - Expense categorization
   - Financial reporting

3. **Multi-Platform Social Media**
   - Facebook posting and monitoring
   - Instagram posting and monitoring
   - Twitter/X posting and monitoring
   - LinkedIn posting and monitoring
   - Engagement tracking and analytics

4. **Multiple MCP Servers**
   - Email MCP (Gmail integration)
   - Social Media MCP (multi-platform)
   - Odoo MCP (accounting integration)
   - Browser MCP (web automation)

5. **Weekly CEO Briefing**
   - Financial performance analysis
   - Task completion metrics
   - Subscription audit
   - Social media analytics
   - Proactive suggestions
   - Upcoming deadlines

6. **Error Recovery & Graceful Degradation**
   - Exponential backoff retry logic
   - Transient error handling
   - Service unavailability fallbacks
   - Comprehensive error logging

7. **Comprehensive Audit Logging**
   - All actions logged to `/Logs/YYYY-MM-DD.json`
   - 90-day retention policy
   - Full audit trail for compliance

8. **Ralph Wiggum Autonomous Loops**
   - Multi-step task completion
   - Iterative execution until done
   - Completion criteria validation
   - Safety limits (max iterations, timeout)

9. **Complete Documentation**
   - Architecture overview
   - Setup instructions
   - Usage guides
   - Lessons learned

10. **Agent Skills Implementation**
    - `/vault-manager`: Task processing
    - `/ceo-briefing`: Weekly business audit
    - `/social-poster`: Multi-platform posting

---

## Directory Structure

```
Gold/
├── .claude/
│   └── skills/
│       ├── vault-manager/
│       │   └── SKILL.md
│       ├── ceo-briefing/
│       │   └── SKILL.md
│       └── social-poster/
│           └── SKILL.md
├── AI_Employee_Vault/
│   ├── vault/
│   │   ├── .obsidian/
│   │   ├── Inbox/
│   │   ├── Needs_Action/
│   │   │   ├── email/
│   │   │   ├── whatsapp/
│   │   │   ├── linkedin/
│   │   │   ├── facebook/
│   │   │   ├── instagram/
│   │   │   ├── twitter/
│   │   │   ├── odoo/
│   │   │   └── filesystem/
│   │   ├── In_Progress/
│   │   ├── Done/
│   │   ├── Pending_Approval/
│   │   ├── Approved/
│   │   ├── Rejected/
│   │   ├── Plans/
│   │   ├── Logs/
│   │   ├── Briefings/
│   │   ├── Accounting/
│   │   ├── Dashboard.md
│   │   ├── Company_Handbook.md
│   │   └── Business_Goals.md
│   ├── watchers/
│   │   ├── base_watcher.py
│   │   ├── odoo_watcher.py
│   │   ├── social_media_watcher.py
│   │   ├── gmail_watcher.py
│   │   ├── whatsapp_watcher.py
│   │   ├── linkedin_watcher.py
│   │   └── filesystem_watcher.py
│   ├── mcp_servers/
│   │   ├── odoo/
│   │   ├── social_media/
│   │   ├── email/
│   │   └── browser/
│   ├── scripts/
│   │   ├── setup.py
│   │   └── test_system.py
│   └── orchestrator.py
├── Drop_Folder/
├── .env.example
├── .gitignore
├── README.md
├── requirements.txt
└── pyproject.toml
```

---

## Prerequisites

### Required Software

| Component | Version | Purpose |
|-----------|---------|---------|
| [Claude Code](https://claude.com/product/claude-code) | Latest | Reasoning engine |
| [Obsidian](https://obsidian.md/download) | v1.10.6+ | Knowledge base |
| [Python](https://www.python.org/downloads/) | 3.13+ | Watchers & scripts |
| [Node.js](https://nodejs.org/) | v24+ LTS | MCP servers |
| [Git](https://git-scm.com/) | Latest | Version control |

### Optional Software

| Component | Purpose |
|-----------|---------|
| [Odoo Community](https://www.odoo.com/) | Accounting system |
| [PM2](https://pm2.keymetrics.io/) | Process management |

### Hardware Requirements

- **Minimum**: 8GB RAM, 4-core CPU, 20GB free disk
- **Recommended**: 16GB RAM, 8-core CPU, SSD storage
- **Network**: Stable 10+ Mbps connection

---

## Installation

### 1. Clone Repository

```bash
git clone <your-repo-url>
cd Gold
```

### 2. Install Python Dependencies

```bash
# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate

# Activate (Mac/Linux)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configure Environment

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your credentials
# IMPORTANT: Never commit .env to git!
```

### 4. Setup Obsidian Vault

1. Open Obsidian
2. Open folder as vault: `AI_Employee_Vault/vault`
3. Trust the vault
4. Review Dashboard.md

### 5. Configure Claude Code

```bash
# Verify Claude Code installation
claude --version

# Configure MCP servers (if needed)
# Edit ~/.config/claude-code/mcp.json
```

### 6. Start System

```bash
# Start orchestrator (manages all watchers)
python AI_Employee_Vault/orchestrator.py AI_Employee_Vault/vault

# Or start individual watchers
python AI_Employee_Vault/watchers/gmail_watcher.py AI_Employee_Vault/vault
```

---

## Configuration

### Environment Variables

Create `.env` file with:

```bash
# Gmail API
GMAIL_CLIENT_ID=your_client_id
GMAIL_CLIENT_SECRET=your_client_secret

# Odoo
ODOO_URL=http://localhost:8069
ODOO_DB=odoo
ODOO_USERNAME=admin
ODOO_PASSWORD=admin

# Social Media (optional)
FACEBOOK_ACCESS_TOKEN=your_token
INSTAGRAM_ACCESS_TOKEN=your_token
TWITTER_API_KEY=your_key
TWITTER_API_SECRET=your_secret
LINKEDIN_ACCESS_TOKEN=your_token

# System
DRY_RUN=false
LOG_LEVEL=INFO
```

### Company Handbook

Edit `vault/Company_Handbook.md` to customize:
- Approval workflows
- Communication guidelines
- Business rules
- Security policies
- Daily routines

### Business Goals

Edit `vault/Business_Goals.md` to set:
- Revenue targets
- Key metrics
- Active projects
- Social media goals
- Financial goals

---

## Usage

### Daily Operations

```bash
# Process pending tasks
claude /vault-manager

# Generate daily briefing
claude /daily-briefing

# Post to social media
claude /social-poster
```

### Weekly Operations

```bash
# Generate CEO briefing (runs automatically Sunday 8 PM)
claude /ceo-briefing
```

### Manual Task Processing

1. Drop files into `Drop_Folder/`
2. Send emails with important keywords
3. Send WhatsApp messages with urgent keywords
4. System automatically detects and processes

### Approval Workflow

1. Check `Pending_Approval/` folder
2. Review approval request
3. Move to `Approved/` to approve
4. Move to `Rejected/` to reject
5. System executes approved actions

---

## Agent Skills

### /vault-manager

Processes tasks from Needs_Action folder.

```bash
claude /vault-manager
claude /vault-manager --priority high
claude /vault-manager --domain email
```

### /ceo-briefing

Generates weekly business audit and CEO briefing.

```bash
claude /ceo-briefing
```

### /social-poster

Creates and posts social media content.

```bash
claude /social-poster
claude /social-poster --platform linkedin
claude /social-poster --batch 7
```

---

## Monitoring & Maintenance

### Check System Status

```bash
# View orchestrator logs
tail -f AI_Employee_Vault/vault/Logs/orchestrator.log

# View watcher logs
tail -f AI_Employee_Vault/vault/Logs/GmailWatcher.log

# Check dashboard
# Open vault/Dashboard.md in Obsidian
```

### Daily Maintenance

- Review Dashboard.md (2 minutes)
- Process approval queue
- Check for errors in logs

### Weekly Maintenance

- Review CEO briefing
- Analyze metrics
- Update goals as needed
- Rotate credentials

### Monthly Maintenance

- Full system audit
- Update Company_Handbook.md
- Review and optimize workflows
- Security review

---

## Troubleshooting

### Watchers Not Starting

```bash
# Check Python version
python --version  # Should be 3.13+

# Check dependencies
pip list

# Check logs
cat AI_Employee_Vault/vault/Logs/orchestrator.log
```

### Claude Code Not Processing

```bash
# Verify Claude Code
claude --version

# Check vault path
cd AI_Employee_Vault/vault
pwd

# Test skill
claude /vault-manager
```

### Odoo Connection Failed

```bash
# Check Odoo is running
curl http://localhost:8069

# Verify credentials in .env
cat .env | grep ODOO

# Test connection
python -c "import xmlrpc.client; print(xmlrpc.client.ServerProxy('http://localhost:8069/xmlrpc/2/common').version())"
```

---

## Security

### Best Practices

1. **Never commit credentials**
   - Use .env files
   - Add to .gitignore
   - Rotate monthly

2. **Use DRY_RUN mode for testing**
   - Set `DRY_RUN=true` in .env
   - Test without real actions

3. **Review all approvals**
   - Never auto-approve payments
   - Check social media posts
   - Verify email recipients

4. **Monitor logs**
   - Check daily for anomalies
   - Set up alerts for errors
   - Retain logs 90+ days

5. **Backup regularly**
   - Daily vault backups
   - Weekly full system backup
   - Test restore procedures

---

## Performance Metrics

### Target Metrics

| Metric | Target | Current |
|--------|--------|---------|
| System Uptime | 99%+ | 100% ✅ |
| Task Processing | < 5 min | N/A |
| Error Recovery | < 1 min | N/A |
| Approval Rate | > 90% | N/A |
| Audit Compliance | 100% | 100% ✅ |

---

## Roadmap

### Completed (Gold Tier)
- ✅ 8 watchers operational
- ✅ Odoo integration
- ✅ Multi-platform social media
- ✅ CEO briefing system
- ✅ Error recovery
- ✅ Comprehensive logging
- ✅ Agent skills

### Next Steps (Platinum Tier)
- [ ] Cloud deployment (24/7 operation)
- [ ] Work-zone specialization
- [ ] Agent-to-agent communication
- [ ] Advanced security hardening
- [ ] Production monitoring

---

## Contributing

This is a hackathon project. Contributions welcome!

1. Fork the repository
2. Create feature branch
3. Make changes
4. Test thoroughly
5. Submit pull request

---

## License

MIT License - See LICENSE file

---

## Acknowledgments

- **Panaversity** for organizing the hackathon
- **Anthropic** for Claude Code
- **Obsidian** for the knowledge base platform
- **Odoo** for the accounting system

---

## Contact

For questions or support:
- GitHub Issues: [your-repo]/issues
- Email: [your-email]
- Discord: [your-discord]

---

## Lessons Learned

### What Worked Well

1. **Local-first architecture**: Privacy and control
2. **File-based workflows**: Simple and reliable
3. **Human-in-the-loop**: Safety and trust
4. **Obsidian integration**: Excellent UX
5. **Agent skills**: Modular and reusable

### Challenges Faced

1. **API rate limits**: Required careful throttling
2. **Error handling**: Many edge cases
3. **State management**: File-based coordination
4. **Testing**: Difficult to test integrations
5. **Documentation**: Keeping it up to date

### Future Improvements

1. **Better error recovery**: More sophisticated retry logic
2. **Advanced analytics**: ML-based insights
3. **Multi-agent coordination**: Parallel processing
4. **Voice interface**: Natural language control
5. **Mobile app**: Remote monitoring

---

**Built with ❤️ for Personal AI Employee Hackathon 0**

*Last Updated: 2026-02-28*
