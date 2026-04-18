"""
Filesystem Watcher - Gold Tier Implementation
Monitors Drop_Folder for new files
"""
import shutil
from pathlib import Path
from datetime import datetime
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from base_watcher import BaseWatcher

class DropFolderHandler(FileSystemEventHandler):
    """Handler for file system events"""

    def __init__(self, watcher):
        self.watcher = watcher

    def on_created(self, event):
        """Handle file creation events"""
        if event.is_directory:
            return

        source = Path(event.src_path)

        # Ignore hidden files and temp files
        if source.name.startswith('.') or source.name.startswith('~'):
            return

        self.watcher.logger.info(f'New file detected: {source.name}')

        # Process the file
        self.watcher.process_dropped_file(source)


class FilesystemWatcher(BaseWatcher):
    """
    Monitors Drop_Folder for new files using watchdog.
    Real-time monitoring with instant processing.
    """

    def __init__(self, vault_path: str):
        """
        Args:
            vault_path: Path to Obsidian vault
        """
        # Use very short interval since watchdog handles real-time
        super().__init__(vault_path, check_interval=5)

        self.filesystem_folder = self.needs_action / 'filesystem'
        self.filesystem_folder.mkdir(parents=True, exist_ok=True)

        # Drop folder is outside vault
        self.drop_folder = self.vault_path.parent.parent / 'Drop_Folder'
        self.drop_folder.mkdir(parents=True, exist_ok=True)

        self.observer = None

    def check_for_updates(self) -> list:
        """
        This method is not used for filesystem watcher.
        We use watchdog's real-time monitoring instead.
        """
        return []

    def create_action_file(self, item) -> Path:
        """Not used - see process_dropped_file instead"""
        pass

    def process_dropped_file(self, source: Path):
        """Process a file dropped into Drop_Folder"""

        try:
            # Skip if already processed
            if str(source) in self.processed_ids:
                return

            # Get file info
            file_size = source.stat().st_size
            file_type = source.suffix.lower()

            # Determine priority based on file type
            priority = self._calculate_priority(file_type, file_size)

            # Copy file to vault
            dest_filename = f'FILE_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{source.name}'
            dest_path = self.filesystem_folder / dest_filename

            shutil.copy2(source, dest_path)
            self.logger.info(f'Copied file to: {dest_path}')

            # Create metadata file
            metadata = {
                'original_name': source.name,
                'file_type': file_type,
                'file_size': file_size,
                'priority': priority,
                'copied_to': str(dest_path)
            }

            content = self.create_metadata_header('file_drop', metadata)
            content += f"## File Drop: {source.name}\n\n"
            content += f"**Original Name**: {source.name}\n"
            content += f"**Type**: {file_type}\n"
            content += f"**Size**: {self._format_size(file_size)}\n"
            content += f"**Priority**: {priority}\n"
            content += f"**Location**: `{dest_path.relative_to(self.vault_path)}`\n\n"
            content += f"### Suggested Actions\n"
            content += f"- [ ] Review file content\n"
            content += f"- [ ] Extract key information\n"
            content += f"- [ ] Process according to type\n"
            content += f"- [ ] Archive when complete\n"

            # Create metadata file
            meta_path = self.filesystem_folder / f'{dest_filename}.md'
            meta_path.write_text(content, encoding='utf-8')

            # Mark as processed
            self.processed_ids.add(str(source))

            # Log action
            self.log_action('file_processed', {
                'filename': source.name,
                'size': file_size,
                'priority': priority
            })

            # Optionally delete original (or move to processed folder)
            # source.unlink()  # Uncomment to delete original

        except Exception as e:
            self.logger.error(f'Error processing file {source}: {e}')
            self.log_action('error', {
                'error': str(e),
                'file': str(source)
            })

    def _calculate_priority(self, file_type: str, file_size: int) -> str:
        """Calculate file priority based on type and size"""

        # High priority file types
        high_priority_types = ['.pdf', '.docx', '.xlsx', '.csv', '.invoice']
        if file_type in high_priority_types:
            return 'high'

        # Medium priority
        medium_priority_types = ['.txt', '.md', '.doc', '.xls']
        if file_type in medium_priority_types:
            return 'medium'

        # Large files might be important
        if file_size > 1_000_000:  # > 1MB
            return 'medium'

        return 'low'

    def _format_size(self, size: int) -> str:
        """Format file size in human-readable format"""
        for unit in ['B', 'KB', 'MB', 'GB']:
            if size < 1024.0:
                return f"{size:.1f} {unit}"
            size /= 1024.0
        return f"{size:.1f} TB"

    def run(self):
        """Main loop with watchdog observer"""
        self.logger.info(f'Starting FilesystemWatcher')
        self.logger.info(f'Monitoring: {self.drop_folder}')

        # Setup watchdog observer
        event_handler = DropFolderHandler(self)
        self.observer = Observer()
        self.observer.schedule(event_handler, str(self.drop_folder), recursive=False)
        self.observer.start()

        self.logger.info('Watchdog observer started - monitoring in real-time')

        try:
            # Keep running
            while True:
                import time
                time.sleep(self.check_interval)

        except KeyboardInterrupt:
            self.logger.info('Stopping filesystem watcher...')
            self.observer.stop()

        self.observer.join()
        self.logger.info('Filesystem watcher stopped')


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python filesystem_watcher.py <vault_path>")
        sys.exit(1)

    vault_path = sys.argv[1]
    watcher = FilesystemWatcher(vault_path)
    watcher.run()
