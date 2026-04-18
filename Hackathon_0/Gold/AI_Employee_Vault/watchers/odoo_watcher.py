"""
Odoo Accounting Watcher
Gold Tier Implementation

Monitors Odoo accounting system for:
- New invoices
- Payment status changes
- Expense entries
- Account reconciliation needs
"""
import os
import json
from pathlib import Path
from datetime import datetime
from dotenv import load_dotenv
from base_watcher import BaseWatcher

# Load environment variables from .env file
load_dotenv()

class OdooWatcher(BaseWatcher):
    """
    Monitors Odoo accounting system via JSON-RPC API.

    Requires:
    - Odoo Community Edition 19+
    - JSON-RPC API access
    - Environment variables: ODOO_URL, ODOO_DB, ODOO_USERNAME, ODOO_PASSWORD
    """

    def __init__(self, vault_path: str, check_interval: int = 60):
        """
        Args:
            vault_path: Path to Obsidian vault
            check_interval: Check every 1 minute (60 seconds)
        """
        super().__init__(vault_path, check_interval)

        self.odoo_folder = self.needs_action / 'odoo'
        self.odoo_folder.mkdir(parents=True, exist_ok=True)

        self.accounting_folder = self.vault_path / 'Accounting'
        self.accounting_folder.mkdir(parents=True, exist_ok=True)

        # Odoo connection details (from environment)
        self.odoo_url = os.getenv('ODOO_URL', 'http://localhost:8069')
        self.odoo_db = os.getenv('ODOO_DB', 'odoo')
        self.odoo_username = os.getenv('ODOO_USERNAME', '')
        self.odoo_password = os.getenv('ODOO_PASSWORD', '')

        self.uid = None
        self.last_check = {}

    def _authenticate(self):
        """Authenticate with Odoo JSON-RPC API"""
        try:
            import xmlrpc.client

            common = xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/common')
            self.uid = common.authenticate(
                self.odoo_db,
                self.odoo_username,
                self.odoo_password,
                {}
            )

            if self.uid:
                self.logger.info(f'Authenticated with Odoo as user ID: {self.uid}')
                return True
            else:
                self.logger.error('Odoo authentication failed')
                return False

        except Exception as e:
            self.logger.error(f'Odoo authentication error: {e}')
            return False

    def check_for_updates(self) -> list:
        """
        Check Odoo for new transactions, invoices, and accounting events.

        NOTE: This is a template implementation.
        Real implementation requires Odoo server running and configured.
        """

        # For demo purposes, return empty list if not configured
        if not self.odoo_username or not self.odoo_password:
            self.logger.info('Odoo credentials not configured - skipping check')
            return []

        # Authenticate if needed
        if not self.uid:
            if not self._authenticate():
                return []

        items = []

        try:
            # Check for new invoices
            items.extend(self._check_invoices())

            # Check for payment updates
            items.extend(self._check_payments())

            # Check for new expenses
            items.extend(self._check_expenses())

            # Check for reconciliation needs
            items.extend(self._check_reconciliation())

        except Exception as e:
            self.logger.error(f'Error checking Odoo: {e}')
            self.log_action('error', {'error': str(e), 'location': 'check_for_updates'})

        return items

    def _get_odoo_models(self):
        """Get Odoo models proxy for API calls"""
        import xmlrpc.client
        return xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/object')

    def _check_invoices(self):
        """Check for new or updated invoices"""
        items = []
        
        try:
            models = self._get_odoo_models()
            
            # Search for posted invoices (customer invoices and vendor bills)
            invoices = models.execute_kw(
                self.odoo_db, self.uid, self.odoo_password,
                'account.move', 'search_read',
                [[('move_type', 'in', ['out_invoice', 'in_invoice', 'out_refund', 'in_refund']), ('state', '=', 'posted')]],
                {
                    'fields': ['name', 'amount_total', 'amount_residual', 'partner_id', 
                               'invoice_date', 'invoice_date_due', 'state', 'payment_state',
                               'currency_id', 'move_type'],
                    'order': 'invoice_date desc, id desc',
                    'limit': 20
                }
            )
            
            for inv in invoices:
                # Get partner name (it's a tuple [id, name])
                partner_name = inv.get('partner_id', ['Unknown'])[1] if inv.get('partner_id') else 'Unknown'
                
                # Get currency name (tuple [id, name])
                currency_name = inv.get('currency_id', ['USD'])[1] if inv.get('currency_id') else 'USD'
                
                # Determine if it's a customer or vendor invoice
                move_type = inv.get('move_type', 'out_invoice')
                invoice_type = 'Customer Invoice' if move_type == 'out_invoice' else \
                              'Vendor Bill' if move_type == 'in_invoice' else \
                              'Customer Credit Note' if move_type == 'out_refund' else 'Vendor Credit Note'
                
                items.append({
                    'id': f'invoice_{inv["id"]}',
                    'type': 'invoice',
                    'odoo_id': inv['id'],
                    'name': inv.get('name', 'N/A'),  # Invoice number like INV/2026/00003
                    'amount': inv.get('amount_total', 0),
                    'residual': inv.get('amount_residual', 0),
                    'currency': currency_name,
                    'partner': partner_name,
                    'date': inv.get('invoice_date', datetime.now().strftime('%Y-%m-%d')),
                    'due_date': inv.get('invoice_date_due', 'N/A'),
                    'state': inv.get('state', 'draft'),
                    'payment_state': inv.get('payment_state', 'not_paid'),
                    'invoice_type': invoice_type
                })
                
            self.logger.info(f'Found {len(invoices)} invoices')
            
        except Exception as e:
            self.logger.error(f'Error checking invoices: {e}')
        
        return items

    def _check_payments(self):
        """Check for payment status changes"""
        items = []
        
        try:
            models = self._get_odoo_models()
            
            # Search for recent payments
            payments = models.execute_kw(
                self.odoo_db, self.uid, self.odoo_password,
                'account.payment', 'search_read',
                [[('state', '=', 'posted')]],
                {
                    'fields': ['name', 'amount', 'partner_id', 'date', 'payment_type', 'state'],
                    'order': 'date desc',
                    'limit': 10
                }
            )
            
            for pay in payments:
                partner_name = pay.get('partner_id', ['Unknown'])[1] if pay.get('partner_id') else 'Unknown'
                
                items.append({
                    'id': f'payment_{pay["id"]}',
                    'type': 'payment',
                    'odoo_id': pay['id'],
                    'name': pay['name'],
                    'amount': pay.get('amount', 0),
                    'currency': 'USD',
                    'partner': partner_name,
                    'date': pay.get('date', datetime.now().strftime('%Y-%m-%d')),
                    'payment_type': pay.get('payment_type', 'manual'),
                    'state': pay.get('state', 'draft')
                })
                
            self.logger.info(f'Found {len(payments)} payments')
            
        except Exception as e:
            self.logger.error(f'Error checking payments: {e}')
        
        return items

    def _check_expenses(self):
        """Check for new expense entries"""
        items = []
        
        try:
            models = self._get_odoo_models()
            
            # Check if hr.expense model exists
            modules = models.execute_kw(
                self.odoo_db, self.uid, self.odoo_password,
                'ir.module.module', 'search_read',
                [[('name', '=', 'hr_expense'), ('state', '=', 'installed')]],
                {'fields': ['name'], 'limit': 1}
            )
            
            if not modules:
                self.logger.debug('hr.expense module not installed - skipping expense check')
                return []
            
            # Search for expenses (requires hr.expense model)
            expenses = models.execute_kw(
                self.odoo_db, self.uid, self.odoo_password,
                'hr.expense', 'search_read',
                [[('state', 'in', ['approved', 'done'])]],
                {
                    'fields': ['name', 'total_amount', 'employee_id', 'date', 'state'],
                    'order': 'date desc',
                    'limit': 10
                }
            )
            
            for exp in expenses:
                employee_name = exp.get('employee_id', ['Unknown'])[1] if exp.get('employee_id') else 'Unknown'
                
                items.append({
                    'id': f'expense_{exp["id"]}',
                    'type': 'expense',
                    'odoo_id': exp['id'],
                    'name': exp['name'],
                    'amount': exp.get('total_amount', 0),
                    'currency': 'USD',
                    'partner': employee_name,
                    'date': exp.get('date', datetime.now().strftime('%Y-%m-%d')),
                    'state': exp.get('state', 'draft')
                })
                
            self.logger.info(f'Found {len(expenses)} expenses')
            
        except Exception as e:
            self.logger.debug(f'Expense check skipped: {e}')
        
        return items

    def _check_reconciliation(self):
        """Check for accounts needing reconciliation"""
        items = []
        
        try:
            models = self._get_odoo_models()
            
            # Search for bank statements (check for unreconciled items)
            # Note: Field names may vary by Odoo version
            statements = models.execute_kw(
                self.odoo_db, self.uid, self.odoo_password,
                'account.bank.statement', 'search_read',
                [],  # Get all, we'll check balance manually
                {
                    'fields': ['name', 'balance', 'balance_end_real', 'date'],
                    'order': 'date desc',
                    'limit': 5
                }
            )
            
            for stmt in statements:
                balance = stmt.get('balance', 0)
                balance_real = stmt.get('balance_end_real', 0)
                
                # Only report if there's a difference
                if abs(balance - balance_real) > 0.01:
                    items.append({
                        'id': f'reconciliation_{stmt["id"]}',
                        'type': 'reconciliation',
                        'odoo_id': stmt['id'],
                        'name': stmt['name'],
                        'balance': balance,
                        'balance_real': balance_real,
                        'currency': 'USD',
                        'partner': 'Bank',
                        'date': stmt.get('date', datetime.now().strftime('%Y-%m-%d')),
                        'difference': balance - balance_real
                    })
                
            self.logger.info(f'Found {len(items)} statements needing reconciliation')
            
        except Exception as e:
            self.logger.debug(f'Reconciliation check skipped: {e}')
        
        return items

    def create_action_file(self, item) -> Path:
        """Create action file for Odoo accounting item"""

        item_id = item.get('id', 'unknown')
        item_type = item.get('type', 'transaction')

        # Skip if already processed
        if item_id in self.processed_ids:
            return None

        # Create metadata
        metadata = {
            'source': 'odoo',
            'item_type': item_type,
            'odoo_id': item_id,
            'priority': self._calculate_priority(item),
            'amount': item.get('amount', 0),
            'currency': item.get('currency', 'USD')
        }

        # Create content
        content = self.create_metadata_header(f'odoo_{item_type}', metadata)
        content += f"## Odoo {item_type.title()}\n\n"
        content += f"**Type**: {item_type}\n"
        content += f"**Amount**: {item.get('currency', 'USD')} {item.get('amount', 0)}\n"
        content += f"**Date**: {item.get('date', datetime.now().strftime('%Y-%m-%d'))}\n"
        content += f"**Partner**: {item.get('partner', 'Unknown')}\n\n"

        if item_type == 'invoice':
            content += f"### Invoice Details\n"
            content += f"- **Invoice Number**: {item.get('name', 'N/A')}\n"
            content += f"- **Type**: {item.get('invoice_type', 'Invoice')}\n"
            content += f"- **Status**: {item.get('state', 'draft')}\n"
            content += f"- **Payment Status**: {item.get('payment_state', 'not_paid')}\n"
            content += f"- **Due Date**: {item.get('due_date', 'N/A')}\n"
            content += f"- **Amount Due**: {item.get('currency', 'USD')} {item.get('residual', item.get('amount', 0))}\n\n"
        elif item_type == 'payment':
            content += f"### Payment Details\n"
            content += f"- **Payment Method**: {item.get('payment_method', 'N/A')}\n"
            content += f"- **Status**: {item.get('state', 'draft')}\n\n"

        content += f"### Suggested Actions\n"
        content += f"- [ ] Review transaction details\n"
        content += f"- [ ] Verify categorization\n"
        content += f"- [ ] Update accounting records\n"
        content += f"- [ ] Generate report if needed\n"

        # Create file with unique filename (include odoo_id to prevent overwrites)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'ODOO_{item_type.upper()}_{item.get("odoo_id", item_id)}_{timestamp}.md'
        filepath = self.odoo_folder / filename

        filepath.write_text(content, encoding='utf-8')
        self.processed_ids.add(item_id)

        # Also update accounting summary
        self._update_accounting_summary(item)

        return filepath

    def _calculate_priority(self, item) -> str:
        """Calculate priority based on amount and type"""
        amount = abs(float(item.get('amount', 0)))
        item_type = item.get('type', '')

        # High priority for large amounts
        if amount > 500:
            return 'high'

        # High priority for overdue invoices
        if item_type == 'invoice' and item.get('state') == 'overdue':
            return 'high'

        # Medium priority for payments
        if item_type == 'payment':
            return 'medium'

        return 'low'

    def _update_accounting_summary(self, item):
        """Update monthly accounting summary"""
        today = datetime.now()
        month_file = self.accounting_folder / f'{today.strftime("%Y-%m")}_Summary.md'

        # Create or update monthly summary
        if not month_file.exists():
            content = f"# Accounting Summary - {today.strftime('%B %Y')}\n\n"
            content += f"## Transactions\n\n"
        else:
            content = month_file.read_text(encoding='utf-8')

        # Add transaction entry
        entry = f"- **{item.get('date', today.strftime('%Y-%m-%d'))}**: "
        entry += f"{item.get('type', 'transaction').title()} - "
        entry += f"{item.get('currency', 'USD')} {item.get('amount', 0)} - "
        entry += f"{item.get('partner', 'Unknown')}\n"

        content += entry
        month_file.write_text(content, encoding='utf-8')


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python odoo_watcher.py <vault_path>")
        sys.exit(1)

    vault_path = sys.argv[1]
    watcher = OdooWatcher(vault_path)
    watcher.run()
