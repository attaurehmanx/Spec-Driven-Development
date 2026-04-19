#!/usr/bin/env python3
"""
Odoo Accounting MCP Server - Platinum Tier
Accounting System Integration for AI Employee

Connects to Odoo via JSON-RPC for accounting operations.
Integrated with Platinum's Cloud/Local agent architecture.
"""

import os
import sys
import json
import logging
import xmlrpc.client
from typing import Any, Dict, List, Optional
from datetime import datetime, timedelta
from pathlib import Path
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
        'description': 'Get list of invoices from Odoo with optional filters',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'state': {
                    'type': 'string',
                    'description': 'Filter by state: draft, posted, cancel',
                    'enum': ['draft', 'posted', 'cancel'],
                },
                'limit': {
                    'type': 'integer',
                    'description': 'Maximum results (default: 10)',
                    'default': 10,
                },
            },
        },
    },
    {
        'name': 'odoo_create_invoice',
        'description': 'Create a draft invoice in Odoo',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'partner_name': {
                    'type': 'string',
                    'description': 'Customer name',
                },
                'invoice_lines': {
                    'type': 'array',
                    'description': 'Invoice line items',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'description': {'type': 'string'},
                            'quantity': {'type': 'number'},
                            'price_unit': {'type': 'number'},
                        },
                        'required': ['description', 'quantity', 'price_unit'],
                    },
                },
            },
            'required': ['partner_name', 'invoice_lines'],
        },
    },
    {
        'name': 'odoo_get_payments',
        'description': 'Get list of payments from Odoo',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'state': {
                    'type': 'string',
                    'description': 'Filter by state: draft, posted, cancel',
                    'enum': ['draft', 'posted', 'cancel'],
                },
                'limit': {
                    'type': 'integer',
                    'description': 'Maximum results',
                    'default': 10,
                },
            },
        },
    },
    {
        'name': 'odoo_get_financial_summary',
        'description': 'Get financial summary for date range',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'date_from': {'type': 'string'},
                'date_to': {'type': 'string'},
            },
        },
    },
]


class OdooMCPServer:
    """Odoo MCP Server for Platinum Tier"""

    def __init__(self):
        self.url = ODOO_CONFIG['url']
        self.db = ODOO_CONFIG['db']
        self.username = ODOO_CONFIG['username']
        self.password = ODOO_CONFIG['password']
        self.uid = None
        self._authenticate()

    def _authenticate(self):
        """Authenticate with Odoo"""
        try:
            common = xmlrpc.client.ServerProxy(f'{self.url}/xmlrpc/2/common')
            self.uid = common.authenticate(self.db, self.username, self.password, {})
            if self.uid:
                logger.info(f"Authenticated with Odoo as user ID: {self.uid}")
            else:
                logger.error("Odoo authentication failed")
        except Exception as e:
            logger.error(f"Odoo authentication error: {e}")
            self.uid = None

    def _get_models(self):
        """Get Odoo models proxy"""
        return xmlrpc.client.ServerProxy(f'{self.url}/xmlrpc/2/object')

    def get_invoices(self, state: str = 'posted', limit: int = 10) -> List[Dict]:
        """Get invoices from Odoo"""
        if not self.uid:
            return []

        try:
            models = self._get_models()
            invoices = models.execute_kw(
                self.db, self.uid, self.password,
                'account.move', 'search_read',
                [[('move_type', 'in', ['out_invoice', 'in_invoice']), ('state', '=', state)]],
                {
                    'fields': ['name', 'amount_total', 'amount_residual', 'partner_id',
                               'invoice_date', 'state', 'payment_state'],
                    'order': 'invoice_date desc',
                    'limit': limit
                }
            )
            return invoices
        except Exception as e:
            logger.error(f"Error getting invoices: {e}")
            return []

    def create_invoice(self, partner_name: str, invoice_lines: List[Dict],
                      invoice_date: str = None) -> Dict:
        """Create draft invoice"""
        if not self.uid:
            return {'error': 'Not authenticated'}

        try:
            models = self._get_models()

            # Search for partner
            partner_ids = models.execute_kw(
                self.db, self.uid, self.password,
                'res.partner', 'search',
                [[['name', '=', partner_name]]]
            )

            if not partner_ids:
                # Create partner
                partner_id = models.execute_kw(
                    self.db, self.uid, self.password,
                    'res.partner', 'create',
                    [{'name': partner_name}]
                )
            else:
                partner_id = partner_ids[0]

            # Prepare invoice lines
            lines = []
            for line in invoice_lines:
                lines.append((0, 0, {
                    'name': line['description'],
                    'quantity': line['quantity'],
                    'price_unit': line['price_unit'],
                }))

            # Create invoice
            invoice_id = models.execute_kw(
                self.db, self.uid, self.password,
                'account.move', 'create',
                [{
                    'move_type': 'out_invoice',
                    'partner_id': partner_id,
                    'invoice_date': invoice_date or datetime.now().strftime('%Y-%m-%d'),
                    'invoice_line_ids': lines,
                }]
            )

            # Get invoice details
            invoice = models.execute_kw(
                self.db, self.uid, self.password,
                'account.move', 'read',
                [invoice_id]
            )[0]

            return {
                'success': True,
                'invoice_id': invoice_id,
                'name': invoice.get('name', 'Draft'),
                'amount_total': invoice.get('amount_total', 0),
            }

        except Exception as e:
            logger.error(f"Error creating invoice: {e}")
            return {'error': str(e)}

    def get_payments(self, state: str = 'posted', limit: int = 10) -> List[Dict]:
        """Get payments"""
        if not self.uid:
            return []

        try:
            models = self._get_models()
            payments = models.execute_kw(
                self.db, self.uid, self.password,
                'account.payment', 'search_read',
                [[('state', '=', state)]],
                {
                    'fields': ['name', 'amount', 'partner_id', 'date', 'payment_type'],
                    'order': 'date desc',
                    'limit': limit
                }
            )
            return payments
        except Exception as e:
            logger.error(f"Error getting payments: {e}")
            return []

    def get_financial_summary(self, date_from: str = None, date_to: str = None) -> Dict:
        """Get financial summary"""
        if not self.uid:
            return {'error': 'Not authenticated'}

        try:
            # Get invoices
            invoices = self.get_invoices(state='posted', limit=100)
            payments = self.get_payments(state='posted', limit=100)

            total_revenue = sum(inv.get('amount_total', 0) for inv in invoices if inv.get('move_type') == 'out_invoice')
            total_expenses = sum(inv.get('amount_total', 0) for inv in invoices if inv.get('move_type') == 'in_invoice')
            total_payments = sum(pay.get('amount', 0) for pay in payments)

            return {
                'success': True,
                'revenue': total_revenue,
                'expenses': total_expenses,
                'profit': total_revenue - total_expenses,
                'payments_made': total_payments,
                'invoice_count': len(invoices),
            }

        except Exception as e:
            logger.error(f"Error getting financial summary: {e}")
            return {'error': str(e)}


def execute_tool(name: str, arguments: dict, server: OdooMCPServer) -> dict:
    """Execute MCP tool"""
    logger.info(f"Executing tool: {name}")

    try:
        if name == 'odoo_get_invoices':
            return {
                'success': True,
                'invoices': server.get_invoices(
                    state=arguments.get('state', 'posted'),
                    limit=arguments.get('limit', 10)
                )
            }
        elif name == 'odoo_create_invoice':
            return server.create_invoice(
                partner_name=arguments.get('partner_name'),
                invoice_lines=arguments.get('invoice_lines', []),
                invoice_date=arguments.get('invoice_date')
            )
        elif name == 'odoo_get_payments':
            return {
                'success': True,
                'payments': server.get_payments(
                    state=arguments.get('state', 'posted'),
                    limit=arguments.get('limit', 10)
                )
            }
        elif name == 'odoo_get_financial_summary':
            return server.get_financial_summary(
                date_from=arguments.get('date_from'),
                date_to=arguments.get('date_to')
            )
        else:
            return {'error': f'Unknown tool: {name}'}

    except Exception as e:
        logger.error(f"Tool execution error: {e}")
        return {'success': False, 'error': str(e)}


def main():
    """Main MCP server loop"""
    logger.info("Odoo MCP Server starting...")

    server = OdooMCPServer()

    if not server.uid:
        logger.error("Failed to authenticate with Odoo. Check credentials.")
        print(json.dumps({
            'error': 'Odoo authentication failed. Check ODOO_* environment variables.'
        }), flush=True)
        return

    while True:
        try:
            line = input()
            if not line:
                continue

            request = json.loads(line)
            method = request.get('method', '')
            params = request.get('params', {})
            req_id = request.get('id')

            if method == 'tools/list':
                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'result': {'tools': TOOLS},
                }
            elif method == 'tools/call':
                tool_name = params.get('name', '')
                arguments = params.get('arguments', {})
                result = execute_tool(tool_name, arguments, server)

                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'result': {
                        'content': [
                            {
                                'type': 'text',
                                'text': json.dumps(result, indent=2),
                            }
                        ],
                        'isError': result.get('success') is False,
                    },
                }
            else:
                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'error': {
                        'code': -32601,
                        'message': f'Method not found: {method}',
                    },
                }

            print(json.dumps(response), flush=True)

        except json.JSONDecodeError as e:
            logger.error(f"JSON decode error: {e}")
        except EOFError:
            break
        except Exception as e:
            logger.error(f"Server error: {e}")
            print(json.dumps({
                'jsonrpc': '2.0',
                'error': {'code': -32603, 'message': str(e)}
            }, flush=True)


if __name__ == '__main__':
    main()
