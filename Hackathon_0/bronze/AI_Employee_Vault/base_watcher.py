"""
Base Watcher Template for AI Employee
All watchers inherit from this base class
"""
import time
import logging
from pathlib import Path
from abc import ABC, abstractmethod
from datetime import datetime

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

    def __init__(self, vault_path: str, check_interval: int = 60):
        """
        Initialize the watcher

        Args:
            vault_path: Path to the Obsidian vault
            check_interval: Seconds between checks (default: 60)
        """
        self.vault_path = Path(vault_path)
        self.needs_action = self.vault_path / 'Needs_Action'
        self.inbox = self.vault_path / 'Inbox'
        self.check_interval = check_interval
        self.logger = logging.getLogger(self.__class__.__name__)

        # Ensure directories exist
        self.needs_action.mkdir(parents=True, exist_ok=True)
        self.inbox.mkdir(parents=True, exist_ok=True)

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
                    self.logger.info(f'Found {len(items)} new items')
                    for item in items:
                        filepath = self.create_action_file(item)
                        self.logger.info(f'Created action file: {filepath.name}')
                else:
                    self.logger.debug('No new items found')

            except KeyboardInterrupt:
                self.logger.info('Watcher stopped by user')
                break
            except Exception as e:
                self.logger.error(f'Error in watcher loop: {e}', exc_info=True)

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
