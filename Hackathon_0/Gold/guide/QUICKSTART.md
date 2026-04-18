# Quick Start Guide - Gold Tier AI Employee

## 5-Minute Setup

### 1. Install Prerequisites (10 minutes)

```bash
# Install Python 3.13+
# Download from: https://www.python.org/downloads/

# Install Claude Code
# Download from: https://claude.com/product/claude-code

# Install Obsidian
# Download from: https://obsidian.md/download

# Install Node.js (for MCP servers)
# Download from: https://nodejs.org/
```

### 2. Clone and Setup (5 minutes)

```bash
# Navigate to Gold directory
cd E:/hackathon-0/Gold

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Mac/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run setup script
python AI_Employee_Vault/scripts/setup.py
```

### 3. Configure Environment (5 minutes)

```bash
# Copy environment template
cp .env.example .env

# Edit .env with your credentials
# At minimum, set DRY_RUN=true for testing
notepad .env  # Windows
nano .env     # Mac/Linux
```

### 4. Open Vault in Obsidian (2 minutes)

1. Open Obsidian
2. Click "Open folder as vault"
3. Select: `E:/hackathon-0/Gold/AI_Employee_Vault/vault`
4. Trust the vault
5. Review Dashboard.md

### 5. Test the System (5 minutes)

```bash
# Run system tests
python AI_Employee_Vault/scripts/test_system.py AI_Employee_Vault/vault

# Should see all tests pass ✅
```

### 6. Start the System (2 minutes)

```bash
# Start orchestrator (manages all watchers)
python AI_Employee_Vault/orchestrator.py AI_Employee_Vault/vault

# You should see:
# - Starting AI Employee Orchestrator (Gold Tier)
# - Started watcher: filesystem
# - Started watcher: gmail
# - etc.
```

### 7. Test with a File Drop (1 minute)

```bash
# In another terminal, create a test file
echo "Test document" > Drop_Folder/test.txt

# Check Obsidian vault - should see new file in:
# Needs_Action/filesystem/FILE_*_test.txt.md
```

### 8. Process with Claude Code (2 minutes)

```bash
# In another terminal, navigate to vault
cd AI_Employee_Vault/vault

# Process pending tasks
claude /vault-manager

# Claude will:
# - Read the task
# - Create a plan
# - Process the file
# - Move to Done folder
```

---

## Daily Usage

### Morning Routine

```bash
# Generate daily briefing
claude /daily-briefing

# Review Dashboard.md in Obsidian
# Check Pending_Approval folder
```

### Processing Tasks

```bash
# Process all pending tasks
claude /vault-manager

# Process high priority only
claude /vault-manager --priority high
```

### Social Media Posting

```bash
# Generate posts for all platforms
claude /social-poster

# Review in Pending_Approval/social/
# Move to Approved/ to post
```

### Weekly CEO Briefing

```bash
# Generate weekly business audit (runs automatically Sunday 8 PM)
claude /ceo-briefing

# Review in Briefings/ folder
```

---

## Troubleshooting

### Orchestrator Won't Start

```bash
# Check Python version
python --version  # Should be 3.13+

# Check dependencies
pip list

# Check logs
cat AI_Employee_Vault/vault/Logs/orchestrator.log
```

### Watchers Not Detecting

```bash
# Check watcher logs
tail -f AI_Employee_Vault/vault/Logs/FilesystemWatcher.log

# Verify Drop_Folder exists
ls Drop_Folder/

# Test with a file
echo "test" > Drop_Folder/test.txt
```

### Claude Code Not Working

```bash
# Verify installation
claude --version

# Check you're in vault directory
cd AI_Employee_Vault/vault
pwd

# Test skill
claude /vault-manager
```

---

## Next Steps

1. **Configure APIs**: Add Gmail, social media credentials to .env
2. **Setup Odoo**: Install and configure accounting system
3. **Customize Handbook**: Edit Company_Handbook.md for your needs
4. **Set Goals**: Update Business_Goals.md with your targets
5. **Test Workflows**: Process real tasks through the system
6. **Monitor Performance**: Review logs and metrics daily
7. **Iterate**: Refine based on what works

---

## Getting Help

- **Documentation**: See README.md for full details
- **Logs**: Check vault/Logs/ for error messages
- **Test Script**: Run test_system.py to diagnose issues
- **Hackathon**: Join Wednesday research meetings

---

**You're ready to go! 🚀**

The system is now monitoring and ready to process tasks autonomously.
