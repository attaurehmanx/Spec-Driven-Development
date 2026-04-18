#!/usr/bin/env python3
"""
Test Odoo MCP Server Connection
Verifies that the server can connect to Odoo and execute basic operations
"""

import os
import sys
import xmlrpc.client
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
ODOO_URL = os.getenv('ODOO_URL', 'http://localhost:8069')
ODOO_DB = os.getenv('ODOO_DB', 'odoo')
ODOO_USERNAME = os.getenv('ODOO_USERNAME', 'admin')
ODOO_PASSWORD = os.getenv('ODOO_PASSWORD', 'admin')


def test_connection():
    """Test connection to Odoo"""
    print("=" * 60)
    print("ODOO MCP SERVER CONNECTION TEST")
    print("=" * 60)
    print()

    print(f"Testing connection to: {ODOO_URL}")
    print(f"Database: {ODOO_DB}")
    print(f"Username: {ODOO_USERNAME}")
    print()

    try:
        # Connect to common endpoint
        print("1. Connecting to Odoo XML-RPC endpoint...")
        common = xmlrpc.client.ServerProxy(f'{ODOO_URL}/xmlrpc/2/common')

        # Get version info
        version = common.version()
        print(f"   [OK] Connected to Odoo {version['server_version']}")
        print(f"   Server series: {version['server_serie']}")
        print()

        # Authenticate
        print("2. Authenticating...")
        uid = common.authenticate(ODOO_DB, ODOO_USERNAME, ODOO_PASSWORD, {})

        if not uid:
            print("   [FAIL] Authentication failed!")
            print("   Check your credentials in .env file")
            return False

        print(f"   [OK] Authenticated successfully (UID: {uid})")
        print()

        # Test model access
        print("3. Testing model access...")
        models = xmlrpc.client.ServerProxy(f'{ODOO_URL}/xmlrpc/2/object')

        # Test reading partners
        partners = models.execute_kw(
            ODOO_DB, uid, ODOO_PASSWORD,
            'res.partner', 'search_read',
            [[]],
            {'fields': ['name'], 'limit': 1}
        )
        print(f"   [OK] Can read res.partner model")

        # Test reading invoices
        invoices = models.execute_kw(
            ODOO_DB, uid, ODOO_PASSWORD,
            'account.move', 'search_read',
            [[]],
            {'fields': ['name'], 'limit': 1}
        )
        print(f"   [OK] Can read account.move model (invoices)")

        # Test reading payments
        payments = models.execute_kw(
            ODOO_DB, uid, ODOO_PASSWORD,
            'account.payment', 'search_read',
            [[]],
            {'fields': ['name'], 'limit': 1}
        )
        print(f"   [OK] Can read account.payment model")
        print()

        # Summary
        print("4. Testing MCP server tools...")
        sys.path.insert(0, os.path.dirname(__file__))
        from server import OdooMCPServer

        server = OdooMCPServer()
        print(f"   [OK] MCP Server initialized")

        # Test get_partners
        result = server.get_partners('', limit=1)
        print(f"   [OK] get_partners: Found {len(result)} partner(s)")

        # Test get_invoices
        result = server.get_invoices(limit=1)
        print(f"   [OK] get_invoices: Found {len(result)} invoice(s)")

        # Test get_payments
        result = server.get_payments(limit=1)
        print(f"   [OK] get_payments: Found {len(result)} payment(s)")

        print()
        print("=" * 60)
        print("[SUCCESS] ALL TESTS PASSED!")
        print("=" * 60)
        print()
        print("Your Odoo MCP Server is ready to use.")
        print()
        print("Next steps:")
        print("1. Add to Claude Code MCP config:")
        print("   ~/.config/claude-code/mcp.json")
        print()
        print("2. Restart Claude Code")
        print()
        print("3. Test with: 'Show me recent invoices from Odoo'")
        print()

        return True

    except xmlrpc.client.ProtocolError as e:
        print(f"   [FAIL] Protocol Error: {e}")
        print(f"   URL: {e.url}")
        print(f"   Error code: {e.errcode}")
        print()
        print("Troubleshooting:")
        print("- Check if Odoo is running: docker ps | grep odoo")
        print("- Verify ODOO_URL in .env file")
        return False

    except ConnectionRefusedError:
        print(f"   [FAIL] Connection refused to {ODOO_URL}")
        print()
        print("Troubleshooting:")
        print("- Start Odoo: docker-compose up -d")
        print("- Check if port 8069 is accessible")
        return False

    except Exception as e:
        print(f"   [FAIL] Error: {e}")
        print()
        print("Troubleshooting:")
        print("- Check .env file configuration")
        print("- Verify Odoo is running and accessible")
        return False


if __name__ == '__main__':
    success = test_connection()
    sys.exit(0 if success else 1)
