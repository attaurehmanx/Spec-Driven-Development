# Odoo Accounting MCP Server
**Gold Tier - Accounting System Integration**

MCP server that connects Claude Code to Odoo via JSON-RPC for accounting operations, financial reporting, and CEO briefings.

## Features

### 📊 Financial Operations
- **Get Invoices**: Query invoices with filters (state, date range, customer)
- **Create Invoices**: Create draft invoices (requires approval before posting)
- **Get Payments**: Track incoming and outgoing payments
- **Account Balances**: Check balances for receivables, payables, cash accounts
- **Financial Summary**: Generate revenue, expense, and profit reports

### 🎯 CEO Briefing Support
- Weekly financial audit data
- Revenue and expense tracking
- Outstanding receivables/payables
- Cash flow analysis
- Transaction summaries

### 🔒 Security
- Read-only by default (invoices created as drafts)
- All operations logged
- Credentials via environment variables
- Human approval required for posting

## Installation

### 1. Install Dependencies

```bash
cd AI_Employee_Vault/mcp_servers/odoo
pip install -r requirements.txt
```

### 2. Configure Environment

Add to your `.env` file (project root):

```bash
# Odoo Configuration
ODOO_URL=http://localhost:8069
ODOO_DB=odoo
ODOO_USERNAME=admin
ODOO_PASSWORD=admin
```

### 3. Test Connection

```bash
python test_connection.py
```

Expected output:
```
============================================================
ODOO MCP SERVER CONNECTION TEST
============================================================

Testing connection to: http://localhost:8069
Database: odoo
Username: admin

1. Connecting to Odoo XML-RPC endpoint...
   ✓ Connected to Odoo 17.0
   Server series: 17.0

2. Authenticating...
   ✓ Authenticated successfully (UID: 2)

3. Testing model access...
   ✓ Can read res.partner model
   ✓ Can read account.move model (invoices)
   ✓ Can read account.payment model

4. Testing MCP server tools...
   ✓ MCP Server initialized
   ✓ get_partners: Found 1 partner(s)
   ✓ get_invoices: Found 0 invoice(s)
   ✓ get_payments: Found 0 payment(s)

============================================================
✓ ALL TESTS PASSED!
============================================================
```

### 4. Add to Claude Code MCP Config

Edit `~/.config/claude-code/mcp.json` (or `%USERPROFILE%\.config\claude-code\mcp.json` on Windows):

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

**Note**: Adjust the path to match your actual project location.

### 5. Restart Claude Code

```bash
# Exit Claude Code and restart
claude
```

## Available Tools

### 1. odoo_get_invoices

Get list of invoices with optional filters.

**Parameters:**
- `state` (optional): Filter by state (draft, posted, cancel)
- `partner_name` (optional): Filter by customer name
- `date_from` (optional): Start date (YYYY-MM-DD)
- `date_to` (optional): End date (YYYY-MM-DD)
- `limit` (optional): Max results (default: 10)

**Example:**
```
Show me all posted invoices from last month
```

### 2. odoo_create_invoice

Create a draft invoice in Odoo.

**Parameters:**
- `partner_name`: Customer name (will search for existing)
- `invoice_lines`: Array of line items with description, quantity, price_unit
- `invoice_date` (optional): Invoice date (defaults to today)

**Example:**
```
Create an invoice for Acme Corp with:
- Web Development: 40 hours at $100/hour
- Hosting: 1 month at $50
```

### 3. odoo_get_payments

Get list of payments with filters.

**Parameters:**
- `state` (optional): Filter by state (draft, posted, cancel)
- `payment_type` (optional): inbound (received) or outbound (sent)
- `date_from` (optional): Start date
- `date_to` (optional): End date
- `limit` (optional): Max results (default: 10)

**Example:**
```
Show me all payments received this week
```

### 4. odoo_get_account_balance

Get balance for specific account types.

**Parameters:**
- `account_type` (optional): Account type (asset_receivable, liability_payable, asset_cash, liability_credit_card)
- `date` (optional): Date for balance (defaults to today)

**Example:**
```
What's our current accounts receivable balance?
```

### 5. odoo_get_financial_summary

Get comprehensive financial summary for a date range.

**Parameters:**
- `date_from`: Start date (YYYY-MM-DD)
- `date_to`: End date (YYYY-MM-DD)

**Returns:**
- Revenue (invoiced, received, outstanding)
- Expenses (billed, paid, outstanding)
- Profit (gross, net cash flow)
- Invoice and bill counts

**Example:**
```
Generate financial summary for last week
```

### 6. odoo_get_partners

Search for customers/partners by name.

**Parameters:**
- `name`: Partner name to search (partial match)
- `limit` (optional): Max results (default: 5)

**Example:**
```
Find customer named "Acme"
```

## Usage Examples

### CEO Briefing Integration

Add to your `.claude/skills/ceo-briefing/SKILL.md`:

```markdown
## Financial Performance (Odoo)

Use the Odoo MCP server to get financial data:

1. Get last week's financial summary:
   - Use odoo_get_financial_summary with date range
   - Report revenue, expenses, profit
   - Highlight outstanding receivables/payables

2. Check account balances:
   - Accounts receivable (money owed to us)
   - Accounts payable (money we owe)
   - Cash/bank balances

3. Recent transactions:
   - Top 5 recent invoices
   - Recent payments received
   - Any overdue invoices
```

### Odoo Watcher Integration

Add to `AI_Employee_Vault/watchers/odoo_watcher.py`:

```python
def check_overdue_invoices(self):
    """Check for overdue invoices"""
    today = datetime.now().strftime('%Y-%m-%d')

    # This would use the MCP server via Claude Code
    # For direct access, use the OdooMCPServer class
    from mcp_servers.odoo.server import OdooMCPServer

    server = OdooMCPServer()
    invoices = server.get_invoices(state='posted', limit=50)

    overdue = [
        inv for inv in invoices
        if inv.get('invoice_date_due') and inv['invoice_date_due'] < today
        and inv['amount_residual'] > 0
    ]

    if overdue:
        self.create_task(
            domain='odoo',
            title=f"{len(overdue)} Overdue Invoices",
            content=f"Follow up on overdue invoices totaling ${sum(inv['amount_residual'] for inv in overdue):.2f}"
        )
```

## Troubleshooting

### "Authentication failed"

**Solution:**
- Verify credentials in .env file
- Check Odoo is running: `docker ps | grep odoo`
- Test login in Odoo web UI: http://localhost:8069

### "Connection refused"

**Solution:**
```bash
# Start Odoo
docker-compose up -d

# Check logs
docker-compose logs odoo
```

### "Model access denied"

**Solution:**
- Ensure user has accounting access rights
- Login to Odoo web UI
- Go to Settings → Users → Select user
- Grant "Accounting / Billing" access

### MCP Server not showing in Claude Code

**Solution:**
1. Check MCP config path is correct
2. Verify Python path in command
3. Restart Claude Code completely
4. Check Claude Code logs for errors

## Architecture

```
┌─────────────────────────────────────────────────────┐
│              CLAUDE CODE (Reasoning)                │
│  - CEO Briefing skill                               │
│  - Vault Manager skill                              │
│  - Financial analysis                               │
└────────────────────┬────────────────────────────────┘
                     │
                     │ MCP Protocol (JSON-RPC)
                     ▼
┌─────────────────────────────────────────────────────┐
│         ODOO MCP SERVER (This Server)               │
│  - server.py (Python)                               │
│  - 6 accounting tools                               │
│  - JSON-RPC client                                  │
└────────────────────┬────────────────────────────────┘
                     │
                     │ Odoo XML-RPC API
                     ▼
┌─────────────────────────────────────────────────────┐
│              ODOO COMMUNITY 17.0                    │
│  - Accounting module                                │
│  - Invoices, Payments, Accounts                     │
│  - Running on Docker (port 8069)                    │
└─────────────────────────────────────────────────────┘
```

## Security Notes

- **Draft-only**: Invoices created as drafts, require manual posting
- **Read-mostly**: Most operations are read-only queries
- **Credentials**: Never commit .env file
- **Logging**: All operations logged for audit trail
- **Approval**: Financial actions require human approval

## Gold Tier Requirements

This MCP server fulfills Gold Tier Requirement #3:

✅ **Accounting system in Odoo Community** (self-hosted, local)
✅ **Integration via MCP server** using Odoo's JSON-RPC APIs
✅ **Multiple tools** for different accounting operations
✅ **CEO Briefing support** with financial summary generation
✅ **Audit logging** of all operations
✅ **Error handling** and graceful degradation

## References

- [Odoo External API Documentation](https://www.odoo.com/documentation/17.0/developer/reference/external_api.html)
- [MCP Protocol Specification](https://spec.modelcontextprotocol.io/)
- [Gold Tier Requirements](../../Personal%20AI%20Employee%20Hackathon%200_%20Building%20Autonomous%20FTEs%20in%202026.md)

---

**Version**: 1.0
**Author**: AI Employee - Gold Tier
**Odoo Version**: 17.0+
**Status**: ✅ Production Ready
