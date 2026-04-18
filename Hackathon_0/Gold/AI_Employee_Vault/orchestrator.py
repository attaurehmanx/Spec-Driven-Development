"""
Orchestrator - Master Process for AI Employee
Gold Tier Implementation

Manages all watchers, schedules tasks, and coordinates workflows.
"""
import os
import sys
import time
import json
import subprocess
from pathlib import Path
from datetime import datetime, timedelta
import logging
import signal

class Orchestrator:
    """
    Master orchestrator for AI Employee system.
    Manages watchers, schedules tasks, and coordinates workflows.
    """

    def __init__(self, vault_path: str):
        self.vault_path = Path(vault_path)
        self.watchers_path = Path(__file__).parent / 'watchers'
        self.logs_path = self.vault_path / 'Logs'
        self.logs_path.mkdir(parents=True, exist_ok=True)

        self.logger = self._setup_logger()
        self.processes = {}
        self.last_briefing = None
        self.running = True

        # Schedule configuration
        self.schedules = {
            'daily_briefing': {'hour': 8, 'minute': 0},
            'midday_check': {'hour': 12, 'minute': 0},
            'evening_summary': {'hour': 18, 'minute': 0},
            'weekly_ceo_briefing': {'weekday': 6, 'hour': 20, 'minute': 0}  # Sunday 8 PM
        }

        # Watcher configuration
        self.watchers = {
            'filesystem': {'script': 'filesystem_watcher.py', 'enabled': True},
            'gmail': {'script': 'gmail_watcher.py', 'enabled': True},
            'whatsapp': {'script': 'whatsapp_watcher.py', 'enabled': True},
            'linkedin': {'script': 'linkedin_watcher.py', 'enabled': True},
            'facebook': {'script': 'social_media_watcher.py', 'args': ['facebook'], 'enabled': True},
            'instagram': {'script': 'social_media_watcher.py', 'args': ['instagram'], 'enabled': True},
            'twitter': {'script': 'social_media_watcher.py', 'args': ['twitter'], 'enabled': True},
            'odoo': {'script': 'odoo_watcher.py', 'enabled': True}
        }

        # Check intervals (seconds) - all set to 1 minute
        self.check_intervals = {
            'filesystem': 5,      # Real-time via watchdog
            'gmail': 60,
            'whatsapp': 60,
            'linkedin': 60,
            'facebook': 60,
            'instagram': 60,
            'twitter': 60,
            'odoo': 60
        }

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _setup_logger(self):
        """Setup logging"""
        logger = logging.getLogger('Orchestrator')
        logger.setLevel(logging.INFO)

        # Console handler
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        logger.addHandler(ch)

        # File handler
        log_file = self.logs_path / 'orchestrator.log'
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.INFO)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

        return logger

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.info(f'Received signal {signum}, shutting down gracefully...')
        self.running = False
        self.stop_all_watchers()
        sys.exit(0)

    def start_watcher(self, name: str, config: dict):
        """Start a watcher process"""
        if not config.get('enabled', True):
            self.logger.info(f'Watcher {name} is disabled, skipping')
            return

        script_path = self.watchers_path / config['script']
        if not script_path.exists():
            self.logger.warning(f'Watcher script not found: {script_path}')
            return

        try:
            args = [sys.executable, str(script_path), str(self.vault_path)]
            if 'args' in config:
                args.extend(config['args'])

            process = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            self.processes[name] = {
                'process': process,
                'started': datetime.now(),
                'restarts': 0
            }

            self.logger.info(f'Started watcher: {name} (PID: {process.pid})')

        except Exception as e:
            self.logger.error(f'Failed to start watcher {name}: {e}')

    def stop_watcher(self, name: str):
        """Stop a watcher process"""
        if name not in self.processes:
            return

        try:
            process_info = self.processes[name]
            process = process_info['process']

            if process.poll() is None:  # Process is running
                process.terminate()
                process.wait(timeout=5)
                self.logger.info(f'Stopped watcher: {name}')

            del self.processes[name]

        except Exception as e:
            self.logger.error(f'Error stopping watcher {name}: {e}')

    def stop_all_watchers(self):
        """Stop all running watchers"""
        self.logger.info('Stopping all watchers...')
        for name in list(self.processes.keys()):
            self.stop_watcher(name)

    def check_watchers(self):
        """Check if watchers are running and restart if needed"""
        for name, process_info in list(self.processes.items()):
            process = process_info['process']

            if process.poll() is not None:  # Process has terminated
                self.logger.warning(f'Watcher {name} has stopped, restarting...')

                # Limit restarts
                if process_info['restarts'] < 5:
                    self.stop_watcher(name)
                    self.start_watcher(name, self.watchers[name])
                    self.processes[name]['restarts'] = process_info['restarts'] + 1
                else:
                    self.logger.error(f'Watcher {name} has failed too many times, giving up')
                    self.stop_watcher(name)

    def check_schedule(self):
        """Check if any scheduled tasks need to run"""
        now = datetime.now()

        # Daily briefing
        if self._should_run('daily_briefing', now):
            self.logger.info('Running daily briefing...')
            self._run_claude_skill('daily-briefing')

        # Midday check
        if self._should_run('midday_check', now):
            self.logger.info('Running midday check...')
            self._process_pending_tasks()

        # Evening summary
        if self._should_run('evening_summary', now):
            self.logger.info('Running evening summary...')
            self._run_claude_skill('daily-briefing')

        # Weekly CEO briefing
        if self._should_run_weekly('weekly_ceo_briefing', now):
            self.logger.info('Running weekly CEO briefing...')
            self._run_claude_skill('ceo-briefing')

    def _should_run(self, task_name: str, now: datetime) -> bool:
        """Check if a scheduled task should run"""
        schedule = self.schedules.get(task_name)
        if not schedule:
            return False

        # Check if we're at the scheduled time (within 1 minute)
        if now.hour == schedule['hour'] and now.minute == schedule['minute']:
            # Check if we haven't run this in the last hour
            last_run_file = self.logs_path / f'last_{task_name}.txt'
            if last_run_file.exists():
                last_run = datetime.fromisoformat(last_run_file.read_text().strip())
                if now - last_run < timedelta(hours=1):
                    return False

            # Mark as run
            last_run_file.write_text(now.isoformat())
            return True

        return False

    def _should_run_weekly(self, task_name: str, now: datetime) -> bool:
        """Check if a weekly scheduled task should run"""
        schedule = self.schedules.get(task_name)
        if not schedule:
            return False

        # Check if it's the right day and time
        if (now.weekday() == schedule['weekday'] and
            now.hour == schedule['hour'] and
            now.minute == schedule['minute']):

            # Check if we haven't run this in the last 6 days
            last_run_file = self.logs_path / f'last_{task_name}.txt'
            if last_run_file.exists():
                last_run = datetime.fromisoformat(last_run_file.read_text().strip())
                if now - last_run < timedelta(days=6):
                    return False

            # Mark as run
            last_run_file.write_text(now.isoformat())
            return True

        return False

    def _run_claude_skill(self, skill_name: str):
        """Run a Claude Code skill"""
        try:
            # This would normally call Claude Code with the skill
            # For now, just log it
            self.logger.info(f'Would run Claude skill: /{skill_name}')

            # In real implementation:
            # subprocess.run(['claude', f'/{skill_name}'], cwd=self.vault_path)

        except Exception as e:
            self.logger.error(f'Error running skill {skill_name}: {e}')

    def _process_pending_tasks(self):
        """Process pending tasks in Needs_Action folder"""
        needs_action = self.vault_path / 'Needs_Action'
        if not needs_action.exists():
            return

        # Count pending tasks
        pending_files = list(needs_action.rglob('*.md'))
        if pending_files:
            self.logger.info(f'Found {len(pending_files)} pending tasks')
            # Would trigger Claude Code to process them

    def update_dashboard(self):
        """Update dashboard with current status"""
        dashboard_path = self.vault_path / 'Dashboard.md'
        if not dashboard_path.exists():
            return

        try:
            # Count tasks in various folders
            needs_action = len(list((self.vault_path / 'Needs_Action').rglob('*.md')))
            pending_approval = len(list((self.vault_path / 'Pending_Approval').rglob('*.md')))
            done_today = self._count_done_today()

            # Update timestamp
            now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Read current dashboard
            content = dashboard_path.read_text(encoding='utf-8')

            # Update last updated time
            content = content.replace(
                'Last Updated**: 2026-02-28 22:00:00',
                f'Last Updated**: {now}'
            )

            # Update task counts (simple replacement)
            # In real implementation, would parse and update properly

            dashboard_path.write_text(content, encoding='utf-8')

        except Exception as e:
            self.logger.error(f'Error updating dashboard: {e}')

    def _count_done_today(self) -> int:
        """Count tasks completed today"""
        done_folder = self.vault_path / 'Done'
        if not done_folder.exists():
            return 0

        today = datetime.now().date()
        count = 0

        for file in done_folder.rglob('*.md'):
            # Check file modification time
            mtime = datetime.fromtimestamp(file.stat().st_mtime).date()
            if mtime == today:
                count += 1

        return count

    def run(self):
        """Main orchestrator loop"""
        self.logger.info('Starting AI Employee Orchestrator (Gold Tier)')
        self.logger.info(f'Vault path: {self.vault_path}')

        # Start all watchers
        for name, config in self.watchers.items():
            self.start_watcher(name, config)

        self.logger.info('All watchers started, entering main loop')

        # Main loop
        while self.running:
            try:
                # Check watchers are running
                self.check_watchers()

                # Check scheduled tasks
                self.check_schedule()

                # Update dashboard
                self.update_dashboard()

                # Sleep for 60 seconds
                time.sleep(60)

            except Exception as e:
                self.logger.error(f'Error in main loop: {e}')
                time.sleep(60)

        self.logger.info('Orchestrator stopped')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python orchestrator.py <vault_path>")
        sys.exit(1)

    vault_path = sys.argv[1]
    orchestrator = Orchestrator(vault_path)
    orchestrator.run()
