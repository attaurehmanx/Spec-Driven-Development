#!/bin/bash
# scripts/setup-vault-sync.sh
# Setup Git-based vault sync between Cloud and Local

set -e

echo "=== Vault Sync Setup ==="
echo ""

# Get vault path
VAULT_PATH="${VAULT_PATH:-./vault}"
echo "Vault path: $VAULT_PATH"

# Get Git repository URL
echo ""
echo "Enter your Git repository URL for vault sync:"
echo "(This repository will store markdown/state files only)"
echo "(Secrets will be excluded via .gitignore)"
read -p "Git Repository URL: " GIT_REPO_URL

# Initialize vault directory
echo ""
echo "Initializing vault directory..."
mkdir -p "$VAULT_PATH"
cd "$VAULT_PATH"

# Check if repository exists
if git rev-parse --git-dir > /dev/null 2>&1; then
    echo "Git repository already initialized"
    
    # Update remote if different
    CURRENT_REMOTE=$(git remote get-url origin 2>/dev/null || echo "")
    if [ "$CURRENT_REMOTE" != "$GIT_REPO_URL" ]; then
        if [ -n "$CURRENT_REMOTE" ]; then
            git remote set-url origin "$GIT_REPO_URL"
        else
            git remote add origin "$GIT_REPO_URL"
        fi
        echo "Updated remote to: $GIT_REPO_URL"
    fi
else
    echo "Initializing new Git repository..."
    git init
    
    # Create .gitignore
    cat > .gitignore << 'GITIGNORE'
# Secrets - NEVER SYNC
.env
*.env
secrets/
tokens/
credentials/
*.key
*.pem

# Sessions - NEVER SYNC
*.session
*.session-*
whatsapp_session/
session_data/

# Logs - Local only
logs/
*.log
sync-logs/

# OS files
.DS_Store
Thumbs.db
desktop.ini

# Editor files
*.swp
*.swo
*~
.vscode/
.idea/
*.sublime-*

# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
venv/
env/
ENV/

# Node
node_modules/
npm-debug.log
yarn-error.log

# Temporary files
*.tmp
*.temp
.cache/
GITIGNORE
    
    # Create initial folder structure
    mkdir -p Inbox Needs_Action/cloud Needs_Action/local
    mkdir -p In_Progress/cloud_agent In_Progress/local_agent
    mkdir -p Pending_Approval Approved Rejected
    mkdir -p Done/cloud Done/local
    mkdir -p Updates Signals Logs
    
    # Initial commit
    git add .
    git commit -m "Initial vault structure"
    
    # Add remote
    git remote add origin "$GIT_REPO_URL"
fi

# Configure git
git config user.name "ai-employee"
git config user.email "ai-employee@local"

# Test connection to remote
echo ""
echo "Testing connection to remote repository..."
if git ls-remote origin > /dev/null 2>&1; then
    echo "✓ Remote repository accessible"
    
    # Pull if remote has content
    if [ "$(git ls-remote origin | wc -l)" -gt 0 ]; then
        echo "Pulling from remote..."
        git pull origin main || git pull origin master || echo "Remote may be empty"
    fi
else
    echo "Warning: Cannot connect to remote repository"
    echo "Will push when remote is available"
fi

# Create .env file for sync configuration
echo ""
echo "Creating sync configuration..."
cat > .env << ENVFILE
GIT_REPO_URL=$GIT_REPO_URL
GIT_EMAIL=ai-employee@local
VAULT_PATH=$VAULT_PATH
SYNC_INTERVAL=300
ENVFILE

echo "✓ Configuration saved to .env"

# Setup complete
echo ""
echo "=== Vault Sync Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. On Cloud VM: Run this script with same Git repository URL"
echo "2. Start sync: python shared/vault_sync/git_sync.py"
echo "3. Or use Docker: docker-compose up -d vault-sync"
echo ""
echo "Security reminder:"
echo "- .env files are excluded from sync (never commit secrets)"
echo "- WhatsApp sessions stay local (never sync to Cloud)"
echo "- Banking credentials stay local (never sync to Cloud)"
echo ""
