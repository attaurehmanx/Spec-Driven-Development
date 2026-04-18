#!/bin/bash

# AI Employee - Quick Start Script
# This script helps you get started with the AI Employee system

echo "=================================="
echo "AI Employee - Bronze Tier Setup"
echo "=================================="
echo ""

# Check if we're in the right directory
if [ ! -d "vault" ]; then
    echo "Error: Please run this script from the AI_Employee_Vault directory"
    exit 1
fi

# Create Drop_Folder if it doesn't exist
if [ ! -d "../Drop_Folder" ]; then
    echo "Creating Drop_Folder..."
    mkdir -p ../Drop_Folder
    echo "✓ Drop_Folder created"
else
    echo "✓ Drop_Folder already exists"
fi

# Check Python installation
echo ""
echo "Checking Python installation..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo "✓ $PYTHON_VERSION found"
else
    echo "✗ Python 3 not found. Please install Python 3.13 or higher"
    exit 1
fi

# Check if requirements are installed
echo ""
echo "Checking Python dependencies..."
if python3 -c "import watchdog" 2>/dev/null; then
    echo "✓ Dependencies already installed"
else
    echo "Installing dependencies..."
    pip install -r requirements.txt
    if [ $? -eq 0 ]; then
        echo "✓ Dependencies installed successfully"
    else
        echo "✗ Failed to install dependencies"
        exit 1
    fi
fi

# Check Claude Code installation
echo ""
echo "Checking Claude Code installation..."
if command -v claude &> /dev/null; then
    echo "✓ Claude Code is installed"
else
    echo "⚠ Claude Code not found. Please install from: https://claude.com/product/claude-code"
fi

echo ""
echo "=================================="
echo "Setup Complete!"
echo "=================================="
echo ""
echo "Next Steps:"
echo ""
echo "1. Start the File System Watcher:"
echo "   python3 filesystem_watcher.py"
echo ""
echo "2. In another terminal, start Claude Code:"
echo "   cd $(pwd)"
echo "   claude"
echo ""
echo "3. Test the system:"
echo "   - Drop a file into Drop_Folder"
echo "   - Use /vault-manager to process tasks"
echo "   - Use /daily-briefing to generate reports"
echo ""
echo "4. Open Obsidian:"
echo "   - Open this folder as a vault"
echo "   - View Dashboard.md for system status"
echo ""
echo "=================================="
