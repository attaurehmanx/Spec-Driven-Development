# CEO Briefing - Odoo Integration Guide

## Adding Financial Data to CEO Briefing

This guide shows how to integrate Odoo financial data into your weekly CEO briefing.

## Update CEO Briefing Skill

Edit `.claude/skills/ceo-briefing/SKILL.md` and add this section:

```markdown
## 3. Financial Performance (Odoo Accounting)

Use the Odoo MCP server to analyze financial performance:

### 3.1 Weekly Financial Summary

Get comprehensive financial data for the past 7 days:

1. Use `odoo_get_financial_summary` with date range (last 7 days)
2. Report the following metrics:
   - **Revenue:**
     - Total invoiced
     - Total received (cash in)
     - Outstanding receivables
   - **Expenses:**
     - Total billed
     - Total paid (cash out)
     - Outstanding payables
   - **Profit:**
     - Gross profit (revenue - expenses)
     - Net cash flow (received - paid)
   - **Activity:**
     - Number of invoices issued
     - Number of bills received

### 3.2 Account Balances

Check current balances for key accounts:

1. Use `odoo_get_account_balance` for:
   - **Accounts Receivable** (money customers owe us)
   - **Accounts Payable** (money we owe suppliers)
   - **Cash/Bank** (available cash)

2. Compare to previous week if possible

### 3.3 Recent Transactions

Highlight important transactions:

1. Use `odoo_get_invoices` with filters:
   - Last 7 days
   - State: posted
   - Limit: 5 largest invoices

2. Use `odoo_get_payments` with filters:
   - Last 7 days
   - Type: inbound (received)
   - Limit: 5 largest payments

### 3.4 Financial Health Indicators

Analyze and report:

1. **Cash Flow Status:**
   - ✅ Positive: Net cash flow > 0
   - ⚠️ Negative: Net cash flow < 0
   - Trend compared to previous weeks

2. **Outstanding Receivables:**
   - Total amount outstanding
   - Percentage of total revenue
   - Any overdue invoices (due date < today)

3. **Outstanding Payables:**
   - Total amount outstanding
   - Any overdue bills
   - Payment schedule for upcoming week

4. **Activity Level:**
   - Invoices issued this week vs average
   - Revenue this week vs average
   - Any unusual patterns

### 3.5 Recommendations

Based on financial data, provide actionable recommendations:

- If outstanding receivables > 30% of revenue:
  → "Follow up with customers on overdue invoices"

- If cash flow negative:
  → "Review expenses and accelerate collections"

- If no invoices issued:
  → "Review sales pipeline and billing schedule"

- If large payments due:
  → "Ensure sufficient cash for upcoming payments"

### Example Output Format

```
## 💰 Financial Performance

**Period:** March 5-12, 2026

### Summary
- Revenue (Invoiced): $15,000
- Revenue (Received): $12,000
- Outstanding: $3,000
- Expenses (Billed): $8,000
- Expenses (Paid): $7,500
- Outstanding: $500
- **Gross Profit:** $7,000
- **Net Cash Flow:** +$4,500 ✅

### Account Balances
- Accounts Receivable: $8,500
- Accounts Payable: $2,300
- Cash/Bank: $25,000

### Top Transactions
1. Invoice INV/2026/0042 - Acme Corp - $5,000 (Paid)
2. Invoice INV/2026/0043 - TechStart Inc - $3,500 (Outstanding)
3. Payment CUST.IN/2026/0015 - Acme Corp - $5,000

### 💡 Insights
✅ Positive cash flow of $4,500 this week
⚠️ $3,000 in outstanding receivables (20% of revenue)
→ Follow up with TechStart Inc on Invoice INV/2026/0043

### Recommendations
1. Contact TechStart Inc regarding outstanding invoice
2. Continue current expense management (on track)
3. Cash position healthy - consider investment opportunities
```
```

## Integration with Odoo Watcher

Add financial monitoring to your Odoo watcher:

### File: `AI_Employee_Vault/watchers/odoo_watcher.py`

Add these methods:

```python
def check_overdue_invoices(self):
    """Check for overdue invoices and create tasks"""
    from datetime import datetime
    import sys
    import os

    # Import Odoo MCP server
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../mcp_servers/odoo'))
    from server import OdooMCPServer

    try:
        server = OdooMCPServer()
        today = datetime.now().strftime('%Y-%m-%d')

        # Get all posted invoices
        invoices = server.get_invoices(state='posted', limit=100)

        # Filter overdue
        overdue = [
            inv for inv in invoices
            if inv.get('invoice_date_due')
            and inv['invoice_date_due'] < today
            and inv['amount_residual'] > 0
        ]

        if overdue:
            total_overdue = sum(inv['amount_residual'] for inv in overdue)

            # Create task in vault
            self.create_task(
                domain='odoo',
                title=f"⚠️ {len(overdue)} Overdue Invoices",
                content=f"""
# Overdue Invoices Alert

**Total Overdue:** ${total_overdue:,.2f}
**Number of Invoices:** {len(overdue)}

## Action Required:
1. Review overdue invoices in Odoo
2. Contact customers for payment
3. Consider payment plans for large amounts

## Overdue Invoices:
{chr(10).join(f"- {inv['name']}: {inv['partner_id'][1]} - ${inv['amount_residual']:.2f} (Due: {inv['invoice_date_due']})" for inv in overdue[:5])}
                """.strip(),
                priority='high'
            )

            self.logger.warning(f"Found {len(overdue)} overdue invoices totaling ${total_overdue:,.2f}")

    except Exception as e:
        self.logger.error(f"Error checking overdue invoices: {e}")


def check_low_cash_balance(self, threshold=5000):
    """Alert if cash balance falls below threshold"""
    import sys
    import os

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../mcp_servers/odoo'))
    from server import OdooMCPServer

    try:
        server = OdooMCPServer()

        # Get cash account balances
        cash_accounts = server.get_account_balance(account_type='asset_cash')
        total_cash = sum(acc['balance'] for acc in cash_accounts)

        if total_cash < threshold:
            self.create_task(
                domain='odoo',
                title=f"⚠️ Low Cash Balance Alert",
                content=f"""
# Low Cash Balance

**Current Balance:** ${total_cash:,.2f}
**Threshold:** ${threshold:,.2f}

## Action Required:
1. Review upcoming expenses
2. Accelerate receivables collection
3. Consider short-term financing if needed
4. Postpone non-essential expenses

## Recommendations:
- Follow up on outstanding invoices
- Review payment terms with customers
- Check for any pending payments
                """.strip(),
                priority='high'
            )

            self.logger.warning(f"Low cash balance: ${total_cash:,.2f}")

    except Exception as e:
        self.logger.error(f"Error checking cash balance: {e}")
```

## Automated Weekly Briefing

Create a scheduled task to generate the briefing automatically:

### File: `AI_Employee_Vault/scripts/generate_weekly_briefing.py`

```python
#!/usr/bin/env python3
"""
Generate Weekly CEO Briefing with Odoo Financial Data
Run this script every Sunday evening
"""

import sys
import os
from datetime import datetime, timedelta

# Add MCP server to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../mcp_servers/odoo'))
from server import OdooMCPServer

def generate_briefing():
    """Generate weekly briefing with financial data"""

    # Calculate date range
    today = datetime.now()
    week_ago = today - timedelta(days=7)
    date_from = week_ago.strftime('%Y-%m-%d')
    date_to = today.strftime('%Y-%m-%d')

    # Connect to Odoo
    server = OdooMCPServer()

    # Get financial summary
    summary = server.get_financial_summary(date_from, date_to)

    # Get account balances
    ar_balance = server.get_account_balance(account_type='asset_receivable')
    ap_balance = server.get_account_balance(account_type='liability_payable')
    cash_balance = server.get_account_balance(account_type='asset_cash')

    ar_total = sum(acc['balance'] for acc in ar_balance)
    ap_total = sum(acc['balance'] for acc in ap_balance)
    cash_total = sum(acc['balance'] for acc in cash_balance)

    # Get recent invoices
    invoices = server.get_invoices(date_from=date_from, date_to=date_to, limit=5)

    # Get recent payments
    payments = server.get_payments(
        payment_type='inbound',
        date_from=date_from,
        date_to=date_to,
        limit=5
    )

    # Generate briefing content
    briefing = f"""
# Weekly CEO Briefing
**Period:** {date_from} to {date_to}
**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

---

## 💰 Financial Performance

### Summary
- **Revenue (Invoiced):** ${summary['revenue']['invoiced']:,.2f}
- **Revenue (Received):** ${summary['revenue']['received']:,.2f}
- **Outstanding Receivables:** ${summary['revenue']['outstanding']:,.2f}
- **Expenses (Billed):** ${summary['expenses']['billed']:,.2f}
- **Expenses (Paid):** ${summary['expenses']['paid']:,.2f}
- **Outstanding Payables:** ${summary['expenses']['outstanding']:,.2f}
- **Gross Profit:** ${summary['profit']['gross']:,.2f}
- **Net Cash Flow:** ${summary['profit']['net_cash_flow']:,.2f}

### Account Balances
- **Accounts Receivable:** ${ar_total:,.2f}
- **Accounts Payable:** ${ap_total:,.2f}
- **Cash/Bank:** ${cash_total:,.2f}

### Activity
- **Invoices Issued:** {summary['invoice_count']}
- **Bills Received:** {summary['bill_count']}

### Recent Invoices
{chr(10).join(f"- {inv['name']}: {inv['partner_id'][1]} - ${inv['amount_total']:.2f}" for inv in invoices[:5])}

### Recent Payments
{chr(10).join(f"- {pay['name']}: {pay['partner_id'][1]} - ${pay['amount']:.2f}" for pay in payments[:5])}

---

## 💡 Insights & Recommendations

"""

    # Add insights
    if summary['profit']['net_cash_flow'] > 0:
        briefing += f"✅ Positive cash flow of ${summary['profit']['net_cash_flow']:,.2f}\n"
    else:
        briefing += f"⚠️ Negative cash flow of ${summary['profit']['net_cash_flow']:,.2f}\n"

    if summary['revenue']['outstanding'] > 0:
        pct = (summary['revenue']['outstanding'] / summary['revenue']['invoiced'] * 100) if summary['revenue']['invoiced'] > 0 else 0
        briefing += f"⚠️ Outstanding receivables: ${summary['revenue']['outstanding']:,.2f} ({pct:.1f}% of revenue)\n"

    if summary['invoice_count'] == 0:
        briefing += "⚠️ No invoices issued this week - review sales pipeline\n"

    # Save to vault
    vault_path = os.path.join(os.path.dirname(__file__), '../vault/Briefings')
    os.makedirs(vault_path, exist_ok=True)

    filename = f"CEO_Briefing_{today.strftime('%Y-%m-%d')}.md"
    filepath = os.path.join(vault_path, filename)

    with open(filepath, 'w') as f:
        f.write(briefing)

    print(f"✓ Briefing generated: {filepath}")

if __name__ == '__main__':
    generate_briefing()
```

## Schedule with Cron (Linux/Mac) or Task Scheduler (Windows)

### Linux/Mac:

```bash
# Edit crontab
crontab -e

# Add this line (runs every Sunday at 8 PM)
0 20 * * 0 cd /path/to/Gold && python AI_Employee_Vault/scripts/generate_weekly_briefing.py
```

### Windows Task Scheduler:

1. Open Task Scheduler
2. Create Basic Task
3. Name: "Weekly CEO Briefing"
4. Trigger: Weekly, Sunday, 8:00 PM
5. Action: Start a program
6. Program: `python`
7. Arguments: `AI_Employee_Vault\scripts\generate_weekly_briefing.py`
8. Start in: `E:\hackathon-0\Gold`

---

**Status:** ✅ Ready to integrate

Your CEO briefing now includes comprehensive financial data from Odoo!
