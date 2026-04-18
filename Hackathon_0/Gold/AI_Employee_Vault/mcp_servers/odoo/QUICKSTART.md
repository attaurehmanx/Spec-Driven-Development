# Odoo MCP Server - Quick Setup Guide

## Prerequisites

✅ Odoo running on Docker (port 8069)
✅ Python 3.13+ with pip
✅ Claude Code installed

## Step-by-Step Installation

### Step 1: Install Dependencies

```bash
cd AI_Employee_Vault/mcp_servers/odoo
pip install -r requirements.txt
```

### Step 2: Configure Environment

Your `.env` file should already have Odoo credentials. Verify:

```bash
# In project root .env file
ODOO_URL=http://localhost:8069
ODOO_DB=odoo
ODOO_USERNAME=admin
ODOO_PASSWORD=admin
```

### Step 3: Test Connection

```bash
python test_connection.py
```

**Expected output:**
```
============================================================
ODOO MCP SERVER CONNECTION TEST
============================================================

✓ Connected to Odoo 17.0
✓ Authenticated successfully
✓ Can read all required models
✓ MCP Server initialized

============================================================
✓ ALL TESTS PASSED!
============================================================
```

If you see errors, check:
- Odoo is running: `docker ps | grep odoo`
- Credentials are correct in .env
- Port 8069 is accessible

### Step 4: Add to Claude Code MCP Config

**Windows:**
Edit `%USERPROFILE%\.config\claude-code\mcp.json`

**Mac/Linux:**
Edit `~/.config/claude-code/mcp.json`

Add this configuration:

```json
{
  "mcpServers": {
    "odoo": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/odoo/server.py"],
      "env": {
        "ODOO_URL": "http://localhost:8069",
        "ODOO_DB": "odoo",
        "ODOO_USERNAME": "admin",
        "ODOO_PASSWORD": "admin"
      }
    }
  }
}
```

**Important:** Change the path in `args` to match your actual project location.

### Step 5: Restart Claude Code

Exit Claude Code completely and restart:

```bash
# Exit current session
exit

# Start new session
claude
```

### Step 6: Test in Claude Code

Try these commands:

```
Show me recent invoices from Odoo
```

```
What's our accounts receivable balance?
```

```
Generate a financial summary for last week
```

If the tools are working, you'll see Claude Code using the Odoo MCP server tools.

## Verification Checklist

- [ ] Dependencies installed (`pip install -r requirements.txt`)
- [ ] Environment variables configured in .env
- [ ] Test connection passes (`python test_connection.py`)
- [ ] MCP config added to Claude Code
- [ ] Claude Code restarted
- [ ] Tools accessible in Claude Code session

## Troubleshooting

### "Module not found: dotenv"

```bash
pip install python-dotenv
```

### "Connection refused to localhost:8069"

```bash
# Start Odoo
docker-compose up -d

# Wait 30 seconds for startup
sleep 30

# Test again
python test_connection.py
```

### "Authentication failed"

Check credentials:
```bash
# View current .env settings
cat .env | grep ODOO

# Try logging in via web browser
# http://localhost:8069
# Username: admin
# Password: admin (or your password)
```

### MCP Server not showing in Claude Code

1. Check the path in mcp.json is absolute and correct
2. Verify Python is in your PATH: `python --version`
3. Check Claude Code logs for errors
4. Try using full Python path: `C:\Python313\python.exe` (Windows) or `/usr/bin/python3` (Linux)

## Next Steps

Once the MCP server is working:

1. **Test with example script:**
   ```bash
   python example_usage.py
   ```

2. **Integrate with CEO Briefing:**
   - See: `.claude/skills/ceo-briefing/SKILL.md`
   - Add Odoo financial data to weekly briefing

3. **Add to Odoo Watcher:**
   - Monitor overdue invoices
   - Track payment status
   - Alert on financial issues

## Example Usage

### Get Financial Summary

```python
from server import OdooMCPServer
from datetime import datetime, timedelta

server = OdooMCPServer()

# Last 7 days
today = datetime.now()
week_ago = today - timedelta(days=7)

summary = server.get_financial_summary(
    date_from=week_ago.strftime('%Y-%m-%d'),
    date_to=today.strftime('%Y-%m-%d')
)

print(f"Revenue: ${summary['revenue']['invoiced']:,.2f}")
print(f"Expenses: ${summary['expenses']['billed']:,.2f}")
print(f"Profit: ${summary['profit']['gross']:,.2f}")
```

### Create Draft Invoice

```python
invoice = server.create_invoice(
    partner_name='Acme Corp',
    invoice_lines=[
        {
            'description': 'Consulting Services',
            'quantity': 10,
            'price_unit': 150.00
        }
    ]
)

print(f"Created invoice: {invoice['name']}")
print(f"Amount: ${invoice['amount_total']:.2f}")
```

## Gold Tier Compliance

This MCP server fulfills **Gold Tier Requirement #3**:

✅ Accounting system in Odoo Community (self-hosted, local)
✅ Integration via MCP server using Odoo's JSON-RPC APIs
✅ Multiple tools for different accounting operations
✅ CEO Briefing support with financial data
✅ Comprehensive error handling
✅ Audit logging

---

**Status:** ✅ Ready for Production

Your Odoo accounting system is now integrated with Claude Code via MCP!
