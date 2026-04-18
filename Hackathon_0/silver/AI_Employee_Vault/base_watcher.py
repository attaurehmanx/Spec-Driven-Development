"""
Base Watcher Template for AI Employee - Silver Tier
All watchers inherit from this base class
"""
import time
import logging
from pathlib import Path
from abc import ABC, abstractmethod
from datetime import datetime
import json

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('watcher.log'),
        logging.StreamHandler()
    ]
)

class BaseWatcher(ABC):
    """Base class for all watcher implementations"""

    def __init__(self, vault_path: str, check_interval: int = 60, watcher_type: str = None):
        """
        Initialize the watcher

        Args:
            vault_path: Path to the Obsidian vault
            check_interval: Seconds between checks (default: 60)
            watcher_type: Type of watcher (gmail, whatsapp, linkedin, filesystem)
        """
        self.vault_path = Path(vault_path)
        self.watcher_type = watcher_type or self.__class__.__name__.lower().replace('watcher', '')

        # Create subfolders for this watcher type in all workflow folders
        self.needs_action_root = self.vault_path / 'Needs_Action'
        self.needs_action = self.needs_action_root / self.watcher_type

        self.inbox_root = self.vault_path / 'Inbox'
        self.inbox = self.inbox_root / self.watcher_type

        self.logs_root = self.vault_path / 'Logs'
        self.logs = self.logs_root / self.watcher_type

        self.done_root = self.vault_path / 'Done'
        self.done = self.done_root / self.watcher_type

        self.pending_approval_root = self.vault_path / 'Pending_Approval'
        self.pending_approval = self.pending_approval_root / self.watcher_type

        self.approved_root = self.vault_path / 'Approved'
        self.approved = self.approved_root / self.watcher_type

        self.rejected_root = self.vault_path / 'Rejected'
        self.rejected = self.rejected_root / self.watcher_type

        self.plans_root = self.vault_path / 'Plans'
        self.plans = self.plans_root / self.watcher_type

        self.check_interval = check_interval
        self.logger = logging.getLogger(self.__class__.__name__)

        # Ensure directories exist
        self.needs_action.mkdir(parents=True, exist_ok=True)
        self.inbox.mkdir(parents=True, exist_ok=True)
        self.logs.mkdir(parents=True, exist_ok=True)
        self.done.mkdir(parents=True, exist_ok=True)
        self.pending_approval.mkdir(parents=True, exist_ok=True)
        self.approved.mkdir(parents=True, exist_ok=True)
        self.rejected.mkdir(parents=True, exist_ok=True)
        self.plans.mkdir(parents=True, exist_ok=True)

    @abstractmethod
    def check_for_updates(self) -> list:
        """
        Check for new items to process

        Returns:
            List of new items detected
        """
        pass

    @abstractmethod
    def create_action_file(self, item) -> Path:
        """
        Create a markdown file in Needs_Action folder

        Args:
            item: The item to process

        Returns:
            Path to the created file
        """
        pass

    def run(self):
        """Main loop - continuously check for updates"""
        self.logger.info(f'Starting {self.__class__.__name__}')
        self.logger.info(f'Monitoring vault at: {self.vault_path}')
        self.logger.info(f'Check interval: {self.check_interval} seconds')

        while True:
            try:
                items = self.check_for_updates()
                if items:
                    print(f"[{self.watcher_type.upper()}] Found {len(items)} new item(s)")
                    for item in items:
                        filepath = self.create_action_file(item)
                        self.logger.info(f'Created action file: {filepath.name}')
                        self.log_action('item_detected', {
                            'watcher': self.__class__.__name__,
                            'file': filepath.name
                        })
                else:
                    self.logger.debug('No new items found')

            except KeyboardInterrupt:
                self.logger.info('Watcher stopped by user')
                break
            except Exception as e:
                self.logger.error(f'Error in watcher loop: {e}', exc_info=True)
                self.log_action('error', {
                    'watcher': self.__class__.__name__,
                    'error': str(e)
                })

            time.sleep(self.check_interval)

    def create_metadata_header(self, item_type: str, **kwargs) -> str:
        """
        Create a YAML frontmatter header for markdown files

        Args:
            item_type: Type of item (e.g., 'file_drop', 'email', 'message')
            **kwargs: Additional metadata fields

        Returns:
            Formatted YAML frontmatter string
        """
        header = f"""---
type: {item_type}
created: {datetime.now().isoformat()}
status: pending
"""
        for key, value in kwargs.items():
            header += f"{key}: {value}\n"
        header += "---\n\n"
        return header

    def log_action(self, action_type: str, details: dict):
        """
        Log an action to the daily log file

        Args:
            action_type: Type of action performed
            details: Dictionary of action details
        """
        log_file = self.logs / f"{datetime.now().strftime('%Y-%m-%d')}.json"

        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'action_type': action_type,
            'actor': self.__class__.__name__,
            'details': details
        }

        # Read existing logs
        logs = []
        if log_file.exists():
            try:
                logs = json.loads(log_file.read_text())
            except:
                logs = []

        # Append new log
        logs.append(log_entry)

        # Write back
        log_file.write_text(json.dumps(logs, indent=2))
