"""
Base Watcher - Template for all watchers
Gold Tier Implementation
"""
import sys
import time
import logging
from pathlib import Path
from abc import ABC, abstractmethod
from datetime import datetime
import json

class BaseWatcher(ABC):
    def __init__(self, vault_path: str, check_interval: int = 60):
        base_path = Path(vault_path)
        # Auto-detect if we need to append 'vault' subfolder
        if (base_path / 'vault').exists() and (base_path / 'vault' / 'Needs_Action').exists():
            self.vault_path = base_path / 'vault'
        else:
            self.vault_path = base_path
        self.needs_action = self.vault_path / 'Needs_Action'
        self.logs_path = self.vault_path / 'Logs'
        self.check_interval = check_interval
        
        # Ensure directories exist BEFORE loading processed IDs
        self.needs_action.mkdir(parents=True, exist_ok=True)
        self.logs_path.mkdir(parents=True, exist_ok=True)
        
        # Set processed IDs file path and load
        self.processed_ids_file = self.logs_path / 'processed_ids.json'
        self.processed_ids = self._load_processed_ids()
        
        self.logger = self._setup_logger()

    def _load_processed_ids(self) -> set:
        """Load processed IDs from file to persist across restarts"""
        if self.processed_ids_file and self.processed_ids_file.exists():
            try:
                with open(self.processed_ids_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    # Keep only IDs from last 30 days to prevent unbounded growth
                    cutoff = datetime.now().timestamp() - (30 * 24 * 60 * 60)
                    filtered_ids = {id_val for id_val, ts in data.items() if ts > cutoff}
                    return filtered_ids
            except (json.JSONDecodeError, FileNotFoundError):
                pass
        return set()

    def _save_processed_ids(self):
        """Save processed IDs to file for persistence"""
        if self.processed_ids_file:
            # Store with timestamp for each ID
            data = {id_val: datetime.now().timestamp() for id_val in self.processed_ids}
            with open(self.processed_ids_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)

    def _setup_logger(self):
        """Setup logging for this watcher"""
        logger = logging.getLogger(self.__class__.__name__)
        logger.setLevel(logging.INFO)

        # Console handler with UTF-8 encoding for emoji support
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        # Set UTF-8 encoding for console output
        ch.stream = open(sys.stdout.fileno(), 'w', encoding='utf-8', closefd=False)
        logger.addHandler(ch)

        # File handler with UTF-8 encoding
        log_file = self.logs_path / f'{self.__class__.__name__}.log'
        fh = logging.FileHandler(log_file, encoding='utf-8')
        fh.setLevel(logging.INFO)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

        return logger

    @abstractmethod
    def check_for_updates(self) -> list:
        """Return list of new items to process"""
        pass

    @abstractmethod
    def create_action_file(self, item) -> Path:
        """Create .md file in Needs_Action folder"""
        pass

    def log_action(self, action_type: str, details: dict):
        """Log action to daily log file"""
        today = datetime.now().strftime('%Y-%m-%d')
        log_file = self.logs_path / f'{today}.json'

        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'watcher': self.__class__.__name__,
            'action_type': action_type,
            'details': details
        }

        # Read existing logs
        logs = []
        if log_file.exists():
            try:
                with open(log_file, 'r', encoding='utf-8') as f:
                    logs = json.load(f)
            except json.JSONDecodeError:
                logs = []

        # Append new log
        logs.append(log_entry)

        # Write back
        with open(log_file, 'w', encoding='utf-8') as f:
            json.dump(logs, f, indent=2, ensure_ascii=False)

    def run(self):
        """Main loop - runs continuously"""
        self.logger.info(f'Starting {self.__class__.__name__}')
        self.logger.info(f'Monitoring interval: {self.check_interval} seconds')

        while True:
            try:
                items = self.check_for_updates()

                if items:
                    self.logger.info(f'Found {len(items)} new items to process')

                for item in items:
                    try:
                        filepath = self.create_action_file(item)

                        # Check if file was actually created (not None)
                        if filepath:
                            # Save processed IDs after each successful file creation
                            self._save_processed_ids()

                            # Log the action
                            self.log_action('item_detected', {
                                'file_created': str(filepath),
                                'item_id': str(item.get('id', 'unknown'))
                            })
                        else:
                            self.logger.debug(f'Skipped duplicate item: {item.get("id", "unknown")}')

                    except Exception as e:
                        self.logger.error(f'Error creating action file: {e}')
                        self.log_action('error', {
                            'error': str(e),
                            'item': str(item)
                        })

            except Exception as e:
                self.logger.error(f'Error in main loop: {e}')
                self.log_action('error', {
                    'error': str(e),
                    'location': 'main_loop'
                })

            time.sleep(self.check_interval)

    def create_metadata_header(self, item_type: str, metadata: dict) -> str:
        """Create YAML frontmatter for action files"""
        header = "---\n"
        header += f"type: {item_type}\n"
        header += f"created: {datetime.now().isoformat()}\n"
        header += f"status: pending\n"

        for key, value in metadata.items():
            if isinstance(value, str):
                # Escape quotes in strings
                value = value.replace('"', '\\"')
                header += f'{key}: "{value}"\n'
            else:
                header += f'{key}: {value}\n'

        header += "---\n\n"
        return header
