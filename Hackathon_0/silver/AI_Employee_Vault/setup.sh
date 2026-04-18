#!/bin/bash
# AI Employee Setup Script - Linux/Mac (Silver Tier)
# Run this script to set up your AI Employee system

set -e  # Exit on error

echo "============================================================"
echo "AI Employee - Silver Tier Setup (Linux/Mac)"
echo "============================================================"
echo ""

# Check Python version
echo "[1/6] Checking Python version..."
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 is not installed"
    echo "Please install Python 3.13 or higher"
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
REQUIRED_VERSION="3.13"

if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
    echo "ERROR: Python $REQUIRED_VERSION or higher is required (found $PYTHON_VERSION)"
    exit 1
fi
echo "✓ Python version OK ($PYTHON_VERSION)"

# Install dependencies
echo ""
echo "[2/6] Installing Python dependencies..."
pip3 install -r requirements.txt
echo "✓ Dependencies installed"

# Create vault structure
echo ""
echo "[3/6] Creating vault directory structure..."
mkdir -p vault/{Inbox,Needs_Action,Done,Plans,Logs,Pending_Approval,Approved,Rejected,Briefings,LinkedIn_Drafts}
echo "✓ Vault structure created"

# Create Drop Folder
echo ""
echo "[4/6] Creating Drop Folder..."
cd ..
mkdir -p Drop_Folder
cd AI_Employee_Vault
echo "✓ Drop Folder created"

# Install Playwright
echo ""
echo "[5/6] Setting up Playwright (for WhatsApp watcher)..."
echo "This may take a few minutes..."
if python3 -m playwright install chromium; then
    echo "✓ Playwright installed"
else
    echo "⚠ Playwright installation failed (optional)"
    echo "WhatsApp watcher will not work without it"
fi

# Create .gitignore
echo ""
echo "[6/6] Creating .gitignore..."
cat > .gitignore << 'EOF'
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
venv/
env/

# Credentials
credentials.json
token.json
.env

# Sessions
whatsapp_session/

# Logs
*.log

# OS
.DS_Store
Thumbs.db
EOF
echo "✓ .gitignore created"

echo ""
echo "============================================================"
echo "Setup Complete!"
echo "============================================================"
echo ""
echo "Next Steps:"
echo "1. Open Obsidian and open the 'vault' folder as a vault"
echo "2. Review vault/Company_Handbook.md"
echo "3. Start the file system watcher:"
echo "   python3 filesystem_watcher.py"
echo ""
echo "Optional Setup:"
echo "- Gmail: Place credentials.json in this directory"
echo "- WhatsApp: Run whatsapp_watcher.py and scan QR code"
echo ""
echo "To start all watchers:"
echo "   python3 orchestrator.py"
echo ""
echo "============================================================"
