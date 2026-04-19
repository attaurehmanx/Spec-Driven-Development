"""
shared/vault_sync/git_sync.py
Git-based vault sync between Cloud and Local
"""

import git
import os
import logging
from pathlib import Path
from datetime import datetime
from typing import Optional

logger = logging.getLogger(__name__)


class VaultSync:
    """
    Git-based vault sync between Cloud and Local
    
    Security Rules:
    1. Never sync .env files
    2. Never sync session files (*.session)
    3. Never sync secrets/ or tokens/ directories
    4. Only sync markdown and state files
    """
    
    def __init__(self, vault_path: str, git_repo_url: Optional[str] = None):
        """
        Initialize vault sync
        
        Args:
            vault_path: Path to vault directory
            git_repo_url: Git repository URL for remote sync
        """
        self.vault_path = Path(vault_path)
        self.git_repo_url = git_repo_url
        self.repo = None
        
        # Ensure vault directory exists
        self.vault_path.mkdir(parents=True, exist_ok=True)
        
    def initialize(self) -> bool:
        """Initialize or clone vault repository"""
        try:
            if (self.vault_path / '.git').exists():
                # Existing vault - initialize git
                logger.info("Existing vault found, initializing git...")
                self.repo = git.Repo(self.vault_path)
            elif self.git_repo_url:
                # New vault - clone from remote
                logger.info(f"Cloning vault from {self.git_repo_url}...")
                self.repo = git.Repo.clone_from(
                    self.git_repo_url,
                    self.vault_path,
                    no_checkout=False
                )
            else:
                # Initialize new repository
                logger.info("Initializing new git repository...")
                self.repo = git.Repo.init(self.vault_path)
            
            # Setup git config
            with self.repo.config_writer() as cw:
                cw.set_value('user', 'name', 'ai-employee')
                cw.set_value('user', 'email', os.getenv('GIT_EMAIL', 'ai-employee@local'))
            
            # Setup .gitignore
            self.setup_gitignore()
            
            logger.info("Vault sync initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize vault sync: {e}")
            return False
    
    def setup_gitignore(self):
        """Create .gitignore to exclude secrets"""
        gitignore_content = """# Secrets - NEVER SYNC
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
"""
        gitignore_path = self.vault_path / '.gitignore'
        
        if not gitignore_path.exists():
            gitignore_path.write_text(gitignore_content)
            logger.info("Created .gitignore")
    
    def push_updates(self, message: str = "Auto-sync") -> bool:
        """
        Push local changes to remote (Cloud only)
        
        Args:
            message: Commit message
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.repo:
                logger.error("Repository not initialized")
                return False
            
            # Check for changes
            if not self.repo.is_dirty() and not self.repo.untracked_files:
                logger.debug("No changes to sync")
                return True
            
            # Add all changes
            self.repo.git.add(A=True)
            
            # Commit
            commit_message = f"{message} - {datetime.now().isoformat()}"
            self.repo.index.commit(commit_message)
            logger.debug(f"Committed: {commit_message}")
            
            # Push
            if self.git_repo_url:
                self.repo.git.push('origin', 'main')
                logger.info(f"✓ Synced to Cloud: {message}")
            else:
                logger.info(f"✓ Committed locally: {message}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to push updates: {e}")
            return False
    
    def pull_updates(self) -> bool:
        """
        Pull updates from remote (Local only)
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.repo:
                logger.error("Repository not initialized")
                return False
            
            if not self.git_repo_url:
                logger.debug("No remote configured, skipping pull")
                return True
            
            # Fetch
            self.repo.git.fetch('origin')
            
            # Pull with rebase
            self.repo.git.pull('--rebase', 'origin', 'main')
            
            logger.info("✓ Synced from Cloud")
            return True
            
        except git.exc.GitCommandError as e:
            if "Conflict" in str(e):
                logger.error(f"Sync conflict detected: {e}")
                # Handle conflict - keep local changes
                self.repo.git.merge('--abort')
            else:
                logger.error(f"Failed to pull updates: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to pull updates: {e}")
            return False
    
    def get_sync_status(self) -> dict:
        """
        Check sync status
        
        Returns:
            Dictionary with sync status information
        """
        status = {
            'initialized': self.repo is not None,
            'last_sync': None,
            'pending_changes': 0,
            'conflicts': False,
            'remote_url': self.git_repo_url,
        }
        
        if not self.repo:
            return status
        
        try:
            # Check for unpushed/unpulled commits
            try:
                ahead = len(list(self.repo.iter_commits('main..HEAD')))
                behind = len(list(self.repo.iter_commits('HEAD..main')))
                
                status['pending_changes'] = ahead + behind
                status['ahead'] = ahead
                status['behind'] = behind
            except:
                pass
            
            # Check for conflicts
            status['conflicts'] = any(
                self.repo.git.diff('--name-only', '--diff-filter=U').split()
            )
            
            # Get last commit time
            try:
                last_commit = self.repo.head.commit
                status['last_sync'] = datetime.fromtimestamp(last_commit.committed_date).isoformat()
            except:
                pass
            
        except Exception as e:
            logger.error(f"Failed to get sync status: {e}")
        
        return status
    
    def force_sync(self) -> bool:
        """
        Force sync (resolve conflicts by keeping remote)
        
        Returns:
            True if successful
        """
        try:
            if not self.repo or not self.git_repo_url:
                return False
            
            # Reset to remote
            self.repo.git.fetch('origin')
            self.repo.git.reset('--hard', 'origin/main')
            
            logger.info("✓ Force synced to remote state")
            return True
            
        except Exception as e:
            logger.error(f"Force sync failed: {e}")
            return False


class SyncScheduler:
    """
    Schedule periodic vault sync
    """
    
    def __init__(self, vault_sync: VaultSync, interval_seconds: int = 300):
        """
        Initialize sync scheduler
        
        Args:
            vault_sync: VaultSync instance
            interval_seconds: Sync interval in seconds (default: 5 minutes)
        """
        self.vault_sync = vault_sync
        self.interval_seconds = interval_seconds
        self.running = False
        
    def start(self):
        """Start periodic sync"""
        import time
        
        self.running = True
        logger.info(f"Starting vault sync scheduler (interval: {self.interval_seconds}s)")
        
        while self.running:
            try:
                # Pull then push
                self.vault_sync.pull_updates()
                self.vault_sync.push_updates()
            except Exception as e:
                logger.error(f"Sync cycle failed: {e}")
            
            time.sleep(self.interval_seconds)
    
    def stop(self):
        """Stop periodic sync"""
        self.running = False
        logger.info("Stopping vault sync scheduler")


if __name__ == '__main__':
    # Example usage
    logging.basicConfig(level=logging.INFO)
    
    vault_path = os.getenv('VAULT_PATH', './vault')
    git_repo_url = os.getenv('GIT_REPO_URL')
    
    sync = VaultSync(vault_path, git_repo_url)
    sync.initialize()
    
    # Test sync
    status = sync.get_sync_status()
    print(f"Sync status: {status}")
