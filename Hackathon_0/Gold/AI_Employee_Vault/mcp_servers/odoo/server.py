#!/usr/bin/env python3
"""
Odoo Accounting MCP Server
Gold Tier - Accounting System Integration

Connects Claude Code to Odoo via JSON-RPC for accounting operations.
"""

import os
import sys
import json
import logging
import xmlrpc.client
from typing import Any, Dict, List, Optional
from datetime import datetime, timedelta
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Odoo Configuration
ODOO_CONFIG = {
    'url': os.getenv('ODOO_URL', 'http://localhost:8069'),
    'db': os.getenv('ODOO_DB', 'odoo'),
    'username': os.getenv('ODOO_USERNAME', 'admin'),
    'password': os.getenv('ODOO_PASSWORD', 'admin'),
}

# MCP Tool Definitions
TOOLS = [
    {
        'name': 'odoo_get_invoices',
        'description': 'Get list of invoices from Odoo with optional filters. Returns invoice details including customer, amount, state, and due date.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'state': {
                    'type': 'string',
                    'description': 'Filter by state: draft, posted, cancel',
                    'enum': ['draft', 'posted', 'cancel'],
                },
                'partner_name': {
                    'type': 'string',
                    'description': 'Filter by customer/partner name (partial match)',
                },
                'date_from': {
                    'type': 'string',
                    'description': 'Start date (YYYY-MM-DD)',
                },
                'date_to': {
                    'type': 'string',
                    'description': 'End date (YYYY-MM-DD)',
                },
                'limit': {
                    'type': 'integer',
                    'description': 'Maximum number of results (default: 10)',
                    'default': 10,
                },
            },
        },
    },
    {
        'name': 'odoo_create_invoice',
        'description': 'Create a draft invoice in Odoo. Invoice must be validated manually or via odoo_post_invoice.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'partner_name': {
                    'type': 'string',
                    'description': 'Customer name (will search for existing partner)',
                },
                'invoice_lines': {
                    'type': 'array',
                    'description': 'Invoice line items',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'description': {
                                'type': 'string',
                                'description': 'Line item description',
                            },
                            'quantity': {
                                'type': 'number',
                                'description': 'Quantity',
                            },
                            'price_unit': {
                                'type': 'number',
                                'description': 'Unit price',
                            },
                        },
                        'required': ['description', 'quantity', 'price_unit'],
                    },
                },
                'invoice_date': {
                    'type': 'string',
                    'description': 'Invoice date (YYYY-MM-DD), defaults to today',
                },
            },
            'required': ['partner_name', 'invoice_lines'],
        },
    },
    {
        'name': 'odoo_get_payments',
        'description': 'Get list of payments from Odoo with optional filters.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'state': {
                    'type': 'string',
                    'description': 'Filter by state: draft, posted, cancel',
                    'enum': ['draft', 'posted', 'cancel'],
                },
                'payment_type': {
                    'type': 'string',
                    'description': 'Filter by type: inbound (received), outbound (sent)',
                    'enum': ['inbound', 'outbound'],
                },
                'date_from': {
                    'type': 'string',
                    'description': 'Start date (YYYY-MM-DD)',
                },
                'date_to': {
                    'type': 'string',
                    'description': 'End date (YYYY-MM-DD)',
                },
                'limit': {
                    'type': 'integer',
                    'description': 'Maximum number of results (default: 10)',
                    'default': 10,
                },
            },
        },
    },
    {
        'name': 'odoo_get_account_balance',
        'description': 'Get balance for specific accounts or account types. Useful for checking cash, bank, receivables, payables.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'account_type': {
                    'type': 'string',
                    'description': 'Account type to query',
                    'enum': ['asset_receivable', 'liability_payable', 'asset_cash', 'liability_credit_card'],
                },
                'date': {
                    'type': 'string',
                    'description': 'Date for balance (YYYY-MM-DD), defaults to today',
                },
            },
        },
    },
    {
        'name': 'odoo_get_financial_summary',
        'description': 'Get financial summary including revenue, expenses, profit for a date range. Perfect for CEO briefings.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'date_from': {
                    'type': 'string',
                    'description': 'Start date (YYYY-MM-DD)',
                },
                'date_to': {
                    'type': 'string',
                    'description': 'End date (YYYY-MM-DD)',
                },
            },
            'required': ['date_from', 'date_to'],
        },
    },
    {
        'name': 'odoo_get_partners',
        'description': 'Search for customers/partners in Odoo by name.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'name': {
                    'type': 'string',
                    'description': 'Partner name to search (partial match)',
                },
                'limit': {
                    'type': 'integer',
                    'description': 'Maximum number of results (default: 5)',
                    'default': 5,
                },
            },
            'required': ['name'],
        },
    },
]


class OdooMCPServer:
    """MCP Server for Odoo Accounting Integration"""

    def __init__(self):
        self.config = ODOO_CONFIG
        self.common = None
        self.models = None
        self.uid = None
        self._connect()

    def _connect(self):
        """Connect to Odoo and authenticate"""
        try:
            self.common = xmlrpc.client.ServerProxy(f"{self.config['url']}/xmlrpc/2/common")
            self.models = xmlrpc.client.ServerProxy(f"{self.config['url']}/xmlrpc/2/object")

            # Authenticate
            self.uid = self.common.authenticate(
                self.config['db'],
                self.config['username'],
                self.config['password'],
                {}
            )

            if not self.uid:
                raise Exception("Authentication failed")

            logger.info(f"Connected to Odoo as {self.config['username']}")

        except Exception as e:
            logger.error(f"Failed to connect to Odoo: {e}")
            raise

    def _execute(self, model: str, method: str, domain_or_args, options=None):
        """Execute Odoo method

        Args:
            model: Odoo model name (e.g., 'res.partner')
            method: Method name (e.g., 'search_read')
            domain_or_args: Domain list or args list for the method
            options: Dict with fields, limit, order, etc.
        """
        try:
            # Ensure domain_or_args is a list
            if not isinstance(domain_or_args, list):
                domain_or_args = [domain_or_args]

            # Call execute_kw with proper format
            if options:
                return self.models.execute_kw(
                    self.config['db'],
                    self.uid,
                    self.config['password'],
                    model,
                    method,
                    [domain_or_args],
                    options
                )
            else:
                return self.models.execute_kw(
                    self.config['db'],
                    self.uid,
                    self.config['password'],
                    model,
                    method,
                    [domain_or_args]
                )
        except Exception as e:
            logger.error(f"Odoo execution error: {e}")
            raise

    def get_invoices(self, state=None, partner_name=None, date_from=None, date_to=None, limit=10):
        """Get invoices with filters"""
        domain = [('move_type', 'in', ['out_invoice', 'out_refund'])]

        if state:
            domain.append(('state', '=', state))

        if date_from:
            domain.append(('invoice_date', '>=', date_from))

        if date_to:
            domain.append(('invoice_date', '<=', date_to))

        if partner_name:
            domain.append(('partner_id', 'ilike', partner_name))

        invoices = self._execute(
            'account.move',
            'search_read',
            domain,
            {
                'fields': ['name', 'partner_id', 'invoice_date', 'amount_total', 'amount_residual', 'state', 'invoice_date_due'],
                'limit': limit,
                'order': 'invoice_date desc'
            }
        )

        return invoices

    def create_invoice(self, partner_name, invoice_lines, invoice_date=None):
        """Create draft invoice"""
        # Find partner
        partners = self._execute(
            'res.partner',
            'search_read',
            [('name', 'ilike', partner_name)],
            {'fields': ['id', 'name'], 'limit': 1}
        )

        if not partners:
            raise Exception(f"Partner '{partner_name}' not found")

        partner_id = partners[0]['id']

        # Prepare invoice lines
        lines = []
        for line in invoice_lines:
            lines.append((0, 0, {
                'name': line['description'],
                'quantity': line['quantity'],
                'price_unit': line['price_unit'],
            }))

        # Create invoice
        invoice_data = {
            'partner_id': partner_id,
            'move_type': 'out_invoice',
            'invoice_line_ids': lines,
        }

        if invoice_date:
            invoice_data['invoice_date'] = invoice_date

        invoice_id = self._execute('account.move', 'create', invoice_data)

        # Get created invoice details
        invoice = self._execute(
            'account.move',
            'read',
            [invoice_id],
            ['name', 'partner_id', 'amount_total', 'state']
        )[0]

        return invoice

    def get_payments(self, state=None, payment_type=None, date_from=None, date_to=None, limit=10):
        """Get payments with filters"""
        domain = []

        if state:
            domain.append(('state', '=', state))

        if payment_type:
            domain.append(('payment_type', '=', payment_type))

        if date_from:
            domain.append(('date', '>=', date_from))

        if date_to:
            domain.append(('date', '<=', date_to))

        payments = self._execute(
            'account.payment',
            'search_read',
            domain,
            {
                'fields': ['name', 'partner_id', 'amount', 'payment_type', 'state', 'date'],
                'limit': limit,
                'order': 'date desc'
            }
        )

        return payments

    def get_account_balance(self, account_type=None, date=None):
        """Get account balance"""
        domain = []

        if account_type:
            domain.append(('account_type', '=', account_type))

        if not date:
            date = datetime.now().strftime('%Y-%m-%d')

        accounts = self._execute(
            'account.account',
            'search_read',
            domain,
            {'fields': ['code', 'name', 'account_type']}
        )

        results = []
        for account in accounts:
            # Get balance for this account
            balance = self._execute(
                'account.move.line',
                'read_group',
                [('account_id', '=', account['id']), ('date', '<=', date)],
                ['debit', 'credit', 'balance'],
                []
            )

            if balance:
                results.append({
                    'account_code': account['code'],
                    'account_name': account['name'],
                    'account_type': account['account_type'],
                    'balance': balance[0].get('balance', 0),
                    'debit': balance[0].get('debit', 0),
                    'credit': balance[0].get('credit', 0),
                })

        return results

    def get_financial_summary(self, date_from, date_to):
        """Get financial summary for date range"""
        # Get revenue (posted invoices)
        revenue_invoices = self._execute(
            'account.move',
            'search_read',
            [
                ('move_type', '=', 'out_invoice'),
                ('state', '=', 'posted'),
                ('invoice_date', '>=', date_from),
                ('invoice_date', '<=', date_to)
            ],
            {'fields': ['amount_total']}
        )

        total_revenue = sum(inv['amount_total'] for inv in revenue_invoices)

        # Get expenses (posted bills)
        expense_bills = self._execute(
            'account.move',
            'search_read',
            [
                ('move_type', '=', 'in_invoice'),
                ('state', '=', 'posted'),
                ('invoice_date', '>=', date_from),
                ('invoice_date', '<=', date_to)
            ],
            {'fields': ['amount_total']}
        )

        total_expenses = sum(bill['amount_total'] for bill in expense_bills)

        # Get payments received
        payments_received = self._execute(
            'account.payment',
            'search_read',
            [
                ('payment_type', '=', 'inbound'),
                ('state', '=', 'posted'),
                ('date', '>=', date_from),
                ('date', '<=', date_to)
            ],
            {'fields': ['amount']}
        )

        total_payments_received = sum(p['amount'] for p in payments_received)

        # Get payments sent
        payments_sent = self._execute(
            'account.payment',
            'search_read',
            [
                ('payment_type', '=', 'outbound'),
                ('state', '=', 'posted'),
                ('date', '>=', date_from),
                ('date', '<=', date_to)
            ],
            {'fields': ['amount']}
        )

        total_payments_sent = sum(p['amount'] for p in payments_sent)

        return {
            'period': f"{date_from} to {date_to}",
            'revenue': {
                'invoiced': total_revenue,
                'received': total_payments_received,
                'outstanding': total_revenue - total_payments_received,
            },
            'expenses': {
                'billed': total_expenses,
                'paid': total_payments_sent,
                'outstanding': total_expenses - total_payments_sent,
            },
            'profit': {
                'gross': total_revenue - total_expenses,
                'net_cash_flow': total_payments_received - total_payments_sent,
            },
            'invoice_count': len(revenue_invoices),
            'bill_count': len(expense_bills),
        }

    def get_partners(self, name, limit=5):
        """Search for partners"""
        domain = []
        if name:
            domain = [('name', 'ilike', name)]

        partners = self._execute(
            'res.partner',
            'search_read',
            domain,
            {'fields': ['id', 'name', 'email', 'phone'], 'limit': limit}
        )
        return partners

    def handle_tool_call(self, name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
        """Handle MCP tool call"""
        try:
            if name == 'odoo_get_invoices':
                result = self.get_invoices(**arguments)
                return {'success': True, 'data': result}

            elif name == 'odoo_create_invoice':
                result = self.create_invoice(**arguments)
                return {'success': True, 'data': result, 'message': f"Created invoice: {result['name']}"}

            elif name == 'odoo_get_payments':
                result = self.get_payments(**arguments)
                return {'success': True, 'data': result}

            elif name == 'odoo_get_account_balance':
                result = self.get_account_balance(**arguments)
                return {'success': True, 'data': result}

            elif name == 'odoo_get_financial_summary':
                result = self.get_financial_summary(**arguments)
                return {'success': True, 'data': result}

            elif name == 'odoo_get_partners':
                result = self.get_partners(**arguments)
                return {'success': True, 'data': result}

            else:
                return {'success': False, 'error': f"Unknown tool: {name}"}

        except Exception as e:
            logger.error(f"Tool execution error: {e}")
            return {'success': False, 'error': str(e)}


def main():
    """Main MCP server loop"""
    server = OdooMCPServer()

    # MCP protocol: read from stdin, write to stdout
    for line in sys.stdin:
        try:
            request = json.loads(line)

            if request.get('method') == 'tools/list':
                response = {
                    'jsonrpc': '2.0',
                    'id': request.get('id'),
                    'result': {'tools': TOOLS}
                }

            elif request.get('method') == 'tools/call':
                params = request.get('params', {})
                tool_name = params.get('name')
                arguments = params.get('arguments', {})

                result = server.handle_tool_call(tool_name, arguments)

                response = {
                    'jsonrpc': '2.0',
                    'id': request.get('id'),
                    'result': {'content': [{'type': 'text', 'text': json.dumps(result, indent=2)}]}
                }

            else:
                response = {
                    'jsonrpc': '2.0',
                    'id': request.get('id'),
                    'error': {'code': -32601, 'message': 'Method not found'}
                }

            print(json.dumps(response), flush=True)

        except Exception as e:
            logger.error(f"Request handling error: {e}")
            error_response = {
                'jsonrpc': '2.0',
                'id': request.get('id') if 'request' in locals() else None,
                'error': {'code': -32603, 'message': str(e)}
            }
            print(json.dumps(error_response), flush=True)


if __name__ == '__main__':
    main()
