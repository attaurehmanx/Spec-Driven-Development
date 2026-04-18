"""
File System Watcher for AI Employee
Monitors a drop folder for new files and creates action items
"""
import shutil
from pathlib import Path
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from base_watcher import BaseWatcher
from datetime import datetime

class DropFolderHandler(FileSystemEventHandler):
    """Handler for file system events in the drop folder"""

    def __init__(self, vault_path: str, drop_folder: str):
        """
        Initialize the drop folder handler

        Args:
            vault_path: Path to the Obsidian vault
            drop_folder: Path to the folder to monitor
        """
        self.vault_path = Path(vault_path)
        self.drop_folder = Path(drop_folder)
        self.needs_action = self.vault_path / 'Needs_Action'
        self.inbox = self.vault_path / 'Inbox'

        # Ensure directories exist
        self.needs_action.mkdir(parents=True, exist_ok=True)
        self.inbox.mkdir(parents=True, exist_ok=True)
        self.drop_folder.mkdir(parents=True, exist_ok=True)

    def on_created(self, event):
        """Handle file creation events"""
        if event.is_directory:
            return

        source = Path(event.src_path)

        # Basic filename sanitization - only take the name, no paths
        safe_name = source.name.replace('..', '').replace('/', '').replace('\\', '')

        # Ignore temporary files and hidden files
        if safe_name.startswith('.') or safe_name.startswith('~'):
            return

        print(f"New file detected: {safe_name}")

        # Copy file to Inbox
        dest = self.inbox / safe_name
        try:
            shutil.copy2(source, dest)
            print(f"Copied to Inbox: {dest}")

            # Create metadata file in Needs_Action
            self.create_metadata(safe_name, source, dest)

            # Optionally remove from drop folder
            # source.unlink()

        except Exception as e:
            print(f"Error processing file {safe_name}: {e}")

    def create_metadata(self, safe_name: str, source: Path, dest: Path):
        """Create a metadata markdown file for the dropped file"""
        # Sanitize the stem for the filename to be extra safe
        import re
        safe_stem = re.sub(r'[^\w\s-]', '', source.stem).strip().replace(' ', '_')
        meta_path = self.needs_action / f'FILE_{safe_stem}.md'

        # Get file stats
        stats = source.stat()
        size_kb = stats.st_size / 1024

        # Quote strings in YAML to prevent injection or formatting breakages
        content = f"""---
type: "file_drop"
original_name: "{safe_name}"
size: {size_kb:.2f}
location: "{dest.as_posix()}"
created: "{datetime.now().isoformat()}"
status: "pending"
priority: "medium"
---

## New File Dropped for Processing

**File Name**: {safe_name}
**Size**: {size_kb:.2f} KB
**Location**: `{dest}`
**Detected**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Suggested Actions
- [ ] Review file contents
- [ ] Categorize and tag appropriately
- [ ] Move to appropriate project folder
- [ ] Archive or delete if not needed

## Notes
*Add any relevant notes or context here*
"""

        meta_path.write_text(content, encoding='utf-8')
        print(f"Created metadata: {meta_path.name}")


class FileSystemWatcher(BaseWatcher):
    """File system watcher implementation"""

    def __init__(self, vault_path: str, drop_folder: str, check_interval: int = 5):
        """
        Initialize the file system watcher

        Args:
            vault_path: Path to the Obsidian vault
            drop_folder: Path to the folder to monitor
            check_interval: Seconds between checks (not used for watchdog)
        """
        super().__init__(vault_path, check_interval)
        self.drop_folder = Path(drop_folder)
        self.drop_folder.mkdir(parents=True, exist_ok=True)

        # Set up watchdog observer
        self.event_handler = DropFolderHandler(vault_path, drop_folder)
        self.observer = Observer()
        self.observer.schedule(self.event_handler, str(self.drop_folder), recursive=False)

    def check_for_updates(self) -> list:
        """Not used for watchdog-based monitoring"""
        return []

    def create_action_file(self, item) -> Path:
        """Not used for watchdog-based monitoring"""
        pass

    def run(self):
        """Start the file system watcher"""
        self.logger.info(f'Starting File System Watcher')
        self.logger.info(f'Monitoring folder: {self.drop_folder}')
        self.logger.info(f'Vault path: {self.vault_path}')

        self.observer.start()
        print(f"\n{'='*60}")
        print(f"File System Watcher Started")
        print(f"{'='*60}")
        print(f"Drop Folder: {self.drop_folder}")
        print(f"Vault Path: {self.vault_path}")
        print(f"\nWatching for new files... (Press Ctrl+C to stop)")
        print(f"{'='*60}\n")

        try:
            while True:
                import time
                time.sleep(1)
        except KeyboardInterrupt:
            self.logger.info('File System Watcher stopped by user')
            self.observer.stop()

        self.observer.join()


if __name__ == '__main__':
    import sys

    # Get vault path from command line or use default
    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'
    drop_folder = sys.argv[2] if len(sys.argv) > 2 else '../Drop_Folder'

    # Create and run the watcher
    watcher = FileSystemWatcher(vault_path, drop_folder)
    watcher.run()
