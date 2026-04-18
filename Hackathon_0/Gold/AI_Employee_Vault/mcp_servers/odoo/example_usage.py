#!/usr/bin/env python3
"""
Example: Using Odoo MCP Server for CEO Briefing
Demonstrates how to use the Odoo MCP server to generate financial reports
"""

import sys
import os
from datetime import datetime, timedelta

# Add parent directory to path
sys.path.insert(0, os.path.dirname(__file__))

from server import OdooMCPServer


def generate_weekly_financial_report():
    """Generate weekly financial report for CEO briefing"""

    print("=" * 70)
    print("WEEKLY FINANCIAL REPORT - CEO BRIEFING")
    print("=" * 70)
    print()

    # Initialize server
    server = OdooMCPServer()

    # Calculate date range (last 7 days)
    today = datetime.now()
    week_ago = today - timedelta(days=7)
    date_from = week_ago.strftime('%Y-%m-%d')
    date_to = today.strftime('%Y-%m-%d')

    print(f"Period: {date_from} to {date_to}")
    print()

    # 1. Financial Summary
    print("📊 FINANCIAL SUMMARY")
    print("-" * 70)

    summary = server.get_financial_summary(date_from, date_to)

    print(f"Revenue:")
    print(f"  Invoiced:    ${summary['revenue']['invoiced']:,.2f}")
    print(f"  Received:    ${summary['revenue']['received']:,.2f}")
    print(f"  Outstanding: ${summary['revenue']['outstanding']:,.2f}")
    print()

    print(f"Expenses:")
    print(f"  Billed:      ${summary['expenses']['billed']:,.2f}")
    print(f"  Paid:        ${summary['expenses']['paid']:,.2f}")
    print(f"  Outstanding: ${summary['expenses']['outstanding']:,.2f}")
    print()

    print(f"Profit:")
    print(f"  Gross:       ${summary['profit']['gross']:,.2f}")
    print(f"  Net Cash:    ${summary['profit']['net_cash_flow']:,.2f}")
    print()

    print(f"Activity:")
    print(f"  Invoices:    {summary['invoice_count']}")
    print(f"  Bills:       {summary['bill_count']}")
    print()

    # 2. Recent Invoices
    print("📄 RECENT INVOICES")
    print("-" * 70)

    invoices = server.get_invoices(date_from=date_from, date_to=date_to, limit=5)

    if invoices:
        for inv in invoices:
            status = "✓ Paid" if inv['amount_residual'] == 0 else f"⏳ Due: ${inv['amount_residual']:.2f}"
            print(f"{inv['name']}: {inv['partner_id'][1]} - ${inv['amount_total']:.2f} - {status}")
    else:
        print("No invoices this week")
    print()

    # 3. Recent Payments
    print("💰 RECENT PAYMENTS")
    print("-" * 70)

    payments = server.get_payments(
        payment_type='inbound',
        date_from=date_from,
        date_to=date_to,
        limit=5
    )

    if payments:
        for pay in payments:
            print(f"{pay['name']}: {pay['partner_id'][1]} - ${pay['amount']:.2f} - {pay['date']}")
    else:
        print("No payments received this week")
    print()

    # 4. Account Balances
    print("💵 ACCOUNT BALANCES")
    print("-" * 70)

    # Accounts Receivable
    ar_balance = server.get_account_balance(account_type='asset_receivable')
    ar_total = sum(acc['balance'] for acc in ar_balance)
    print(f"Accounts Receivable: ${ar_total:,.2f}")

    # Accounts Payable
    ap_balance = server.get_account_balance(account_type='liability_payable')
    ap_total = sum(acc['balance'] for acc in ap_balance)
    print(f"Accounts Payable:    ${ap_total:,.2f}")

    # Cash
    cash_balance = server.get_account_balance(account_type='asset_cash')
    cash_total = sum(acc['balance'] for acc in cash_balance)
    print(f"Cash/Bank:           ${cash_total:,.2f}")
    print()

    # 5. Insights & Recommendations
    print("💡 INSIGHTS & RECOMMENDATIONS")
    print("-" * 70)

    # Check for outstanding receivables
    if summary['revenue']['outstanding'] > 0:
        print(f"⚠️  Outstanding receivables: ${summary['revenue']['outstanding']:,.2f}")
        print("   → Follow up with customers on overdue invoices")

    # Check cash flow
    if summary['profit']['net_cash_flow'] < 0:
        print(f"⚠️  Negative cash flow: ${summary['profit']['net_cash_flow']:,.2f}")
        print("   → Review expenses and accelerate collections")
    elif summary['profit']['net_cash_flow'] > 0:
        print(f"✓  Positive cash flow: ${summary['profit']['net_cash_flow']:,.2f}")

    # Check activity
    if summary['invoice_count'] == 0:
        print("⚠️  No invoices issued this week")
        print("   → Review sales pipeline and billing schedule")

    print()
    print("=" * 70)
    print("Report generated:", datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    print("=" * 70)


def test_invoice_creation():
    """Test creating a draft invoice"""

    print("\n" + "=" * 70)
    print("TEST: CREATE DRAFT INVOICE")
    print("=" * 70)
    print()

    server = OdooMCPServer()

    # Search for a partner first
    print("1. Searching for partner...")
    partners = server.get_partners('', limit=1)

    if not partners:
        print("   ✗ No partners found. Create a customer in Odoo first.")
        return

    partner = partners[0]
    print(f"   ✓ Found partner: {partner['name']}")
    print()

    # Create invoice
    print("2. Creating draft invoice...")
    try:
        invoice = server.create_invoice(
            partner_name=partner['name'],
            invoice_lines=[
                {
                    'description': 'Consulting Services',
                    'quantity': 10,
                    'price_unit': 100.00
                },
                {
                    'description': 'Software License',
                    'quantity': 1,
                    'price_unit': 500.00
                }
            ]
        )

        print(f"   ✓ Created invoice: {invoice['name']}")
        print(f"   Customer: {invoice['partner_id'][1]}")
        print(f"   Amount: ${invoice['amount_total']:.2f}")
        print(f"   State: {invoice['state']}")
        print()
        print("   Note: Invoice created as DRAFT. Post it manually in Odoo.")

    except Exception as e:
        print(f"   ✗ Error: {e}")

    print()


if __name__ == '__main__':
    # Generate weekly report
    generate_weekly_financial_report()

    # Optionally test invoice creation
    # Uncomment to test:
    # test_invoice_creation()
