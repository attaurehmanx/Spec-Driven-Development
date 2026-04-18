"""
Orchestrator for AI Employee - Silver Tier
Manages multiple watchers and coordinates their execution
"""
import subprocess
import time
import logging
from pathlib import Path
from datetime import datetime
import json
import sys

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('orchestrator.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger('Orchestrator')

class Orchestrator:
    """Manages multiple watcher processes"""

    def __init__(self, vault_path: str):
        """
        Initialize the orchestrator

        Args:
            vault_path: Path to the Obsidian vault
        """
        self.vault_path = Path(vault_path)
        self.processes = {}
        self.config_file = Path('orchestrator_config.json')
        self.load_config()

    def load_config(self):
        """Load orchestrator configuration"""
        if self.config_file.exists():
            try:
                self.config = json.loads(self.config_file.read_text())
            except:
                self.config = self.get_default_config()
        else:
            self.config = self.get_default_config()
            self.save_config()

    def save_config(self):
        """Save orchestrator configuration"""
        self.config_file.write_text(json.dumps(self.config, indent=2))

    def get_default_config(self):
        """Get default configuration"""
        return {
            'watchers': {
                'filesystem': {
                    'enabled': True,
                    'script': 'filesystem_watcher.py',
                    'args': ['./vault', '../Drop_Folder']
                },
                'linkedin': {
                    'enabled': True,
                    'script': 'linkedin_watcher.py',
                    'args': ['./vault']
                },
                'gmail': {
                    'enabled': True,  # Now configured!
                    'script': 'gmail_watcher.py',
                    'args': ['./vault', 'credentials.json']
                },
                'whatsapp': {
                    'enabled': True,  # Now working!
                    'script': 'whatsapp_watcher_simple.py',  # Use simple version
                    'args': ['./vault', './whatsapp_session']
                },
                'email_sender': {
                    'enabled': True,
                    'script': 'send_approved_email.py',
                    'args': ['./vault', 'credentials.json', 'watch']
                }
            }
        }

    def start_watcher(self, name: str, config: dict):
        """Start a watcher process"""
        if not config.get('enabled', False):
            logger.info(f"Watcher '{name}' is disabled")
            return

        script = config['script']
        args = config.get('args', [])

        try:
            cmd = [sys.executable, script] + args + ['--quiet']  # Add quiet flag for orchestrator
            # Don't capture stdout/stderr - let watchers print directly to terminal
            process = subprocess.Popen(
                cmd,
                # stdout and stderr will go to terminal for real-time visibility
            )
            self.processes[name] = {
                'process': process,
                'config': config,
                'started': datetime.now().isoformat()
            }
            logger.info(f"Started watcher: {name} (PID: {process.pid})")

        except Exception as e:
            logger.error(f"Failed to start watcher '{name}': {e}")

    def stop_watcher(self, name: str):
        """Stop a watcher process"""
        if name in self.processes:
            process_info = self.processes[name]
            process = process_info['process']

            try:
                process.terminate()
                process.wait(timeout=5)
                logger.info(f"Stopped watcher: {name}")
            except:
                process.kill()
                logger.warning(f"Force killed watcher: {name}")

            del self.processes[name]

    def check_health(self):
        """Check health of all running watchers"""
        for name, info in list(self.processes.items()):
            process = info['process']

            if process.poll() is not None:
                logger.warning(f"Watcher '{name}' has stopped (exit code: {process.returncode})")
                # Restart if configured
                if self.config['watchers'][name].get('auto_restart', True):
                    logger.info(f"Restarting watcher: {name}")
                    self.start_watcher(name, self.config['watchers'][name])

    def start_all(self):
        """Start all enabled watchers"""
        logger.info("Starting AI Employee Orchestrator")
        logger.info(f"Vault path: {self.vault_path}")

        for name, config in self.config['watchers'].items():
            self.start_watcher(name, config)

        logger.info(f"Started {len(self.processes)} watchers")

    def stop_all(self):
        """Stop all running watchers"""
        logger.info("Stopping all watchers")

        for name in list(self.processes.keys()):
            self.stop_watcher(name)

        logger.info("All watchers stopped")

    def run(self):
        """Main orchestrator loop"""
        self.start_all()

        print(f"\n{'='*60}")
        print(f"AI Employee Orchestrator Running")
        print(f"{'='*60}")
        print(f"Active Watchers: {len(self.processes)}")
        for name in self.processes.keys():
            print(f"  * {name}")
        print(f"\nMonitoring system health... (Press Ctrl+C to stop)")
        print(f"{'='*60}\n")

        try:
            while True:
                time.sleep(30)  # Check every 30 seconds
                self.check_health()

        except KeyboardInterrupt:
            logger.info("Orchestrator stopped by user")
            self.stop_all()


if __name__ == '__main__':
    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'

    orchestrator = Orchestrator(vault_path)
    orchestrator.run()
