"""
shared/claim_manager/claim_manager.py
Multi-agent coordination via claim-by-move rule
"""

import logging
from pathlib import Path
from datetime import datetime
from typing import List, Optional, Dict, Any

logger = logging.getLogger(__name__)


class ClaimManager:
    """
    Multi-agent coordination via claim-by-move rule
    
    Rule: First agent to move task to their folder owns it
    Other agents must ignore claimed tasks
    """
    
    def __init__(self, vault_path: str, agent_name: str):
        """
        Initialize claim manager
        
        Args:
            vault_path: Path to vault directory
            agent_name: Name of this agent (e.g., 'cloud_agent', 'local_agent')
        """
        self.vault_path = Path(vault_path)
        self.agent_name = agent_name
        
        # Ensure folders exist
        self.needs_action_folder = self.vault_path / 'Needs_Action'
        self.in_progress_folder = self.vault_path / 'In_Progress' / agent_name
        
        self.needs_action_folder.mkdir(parents=True, exist_ok=True)
        self.in_progress_folder.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"ClaimManager initialized for {agent_name}")
    
    def claim_task(self, task_file: Path) -> bool:
        """
        Claim a task by moving it to agent's In_Progress folder
        
        Args:
            task_file: Path to task file in Needs_Action
            
        Returns:
            True if successfully claimed, False if already claimed by another agent
        """
        # Validate task file exists
        if not task_file.exists():
            logger.warning(f"Task file does not exist: {task_file}")
            return False
        
        # Check if task is in Needs_Action
        if task_file.parent != self.needs_action_folder:
            logger.warning(f"Task {task_file} not in Needs_Action folder")
            return False
        
        # Check if already claimed by us
        claimed_file = self.in_progress_folder / task_file.name
        if claimed_file.exists():
            logger.info(f"Task {task_file} already claimed by {self.agent_name}")
            return True
        
        # Check if claimed by another agent
        in_progress_root = self.vault_path / 'In_Progress'
        if in_progress_root.exists():
            for agent_folder in in_progress_root.iterdir():
                if agent_folder.is_dir() and agent_folder.name != self.agent_name:
                    if (agent_folder / task_file.name).exists():
                        logger.info(f"Task {task_file} already claimed by {agent_folder.name}")
                        return False
        
        # Move to our In_Progress folder
        try:
            task_file.rename(claimed_file)
            logger.info(f"✓ Task {task_file.name} claimed by {self.agent_name}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to claim task {task_file}: {e}")
            return False
    
    def release_task(self, task_file: Path, status: str = 'done') -> bool:
        """
        Release a task after completion
        
        Args:
            task_file: Path to task file in In_Progress
            status: 'done', 'rejected', 'failed'
            
        Returns:
            True if successful
        """
        # Determine target folder based on status
        if status == 'done':
            target_folder = self.vault_path / 'Done' / self.agent_name
        elif status == 'rejected':
            target_folder = self.vault_path / 'Rejected'
        elif status == 'failed':
            target_folder = self.vault_path / 'Failed'
        else:
            target_folder = self.vault_path / 'Done' / self.agent_name
        
        # Ensure target folder exists
        target_folder.mkdir(parents=True, exist_ok=True)
        
        # Move task
        try:
            target_file = target_folder / task_file.name
            task_file.rename(target_file)
            logger.info(f"✓ Task {task_file.name} released with status: {status}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to release task {task_file}: {e}")
            return False
    
    def get_available_tasks(self) -> List[Path]:
        """
        Get list of unclaimed tasks in Needs_Action
        
        Returns:
            List of available task file paths
        """
        if not self.needs_action_folder.exists():
            return []
        
        available_tasks = []
        
        for task_file in self.needs_action_folder.glob('*.md'):
            if not self._is_task_claimed(task_file):
                available_tasks.append(task_file)
        
        logger.debug(f"Found {len(available_tasks)} available tasks")
        return available_tasks
    
    def _is_task_claimed(self, task_file: Path) -> bool:
        """
        Check if a task is claimed by any agent
        
        Args:
            task_file: Path to task file
            
        Returns:
            True if claimed
        """
        in_progress_root = self.vault_path / 'In_Progress'
        
        if not in_progress_root.exists():
            return False
        
        for agent_folder in in_progress_root.iterdir():
            if agent_folder.is_dir():
                if (agent_folder / task_file.name).exists():
                    return True
        
        return False
    
    def get_claimed_tasks(self) -> List[Path]:
        """
        Get list of tasks claimed by this agent
        
        Returns:
            List of claimed task file paths
        """
        if not self.in_progress_folder.exists():
            return []
        
        return list(self.in_progress_folder.glob('*.md'))
    
    def get_task_metadata(self, task_file: Path) -> Dict[str, Any]:
        """
        Get metadata from task file frontmatter
        
        Args:
            task_file: Path to task file
            
        Returns:
            Dictionary with metadata
        """
        if not task_file.exists():
            return {}
        
        content = task_file.read_text()
        metadata = {}
        
        # Parse frontmatter (YAML between --- markers)
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 2:
                frontmatter = parts[1].strip()
                for line in frontmatter.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        metadata[key.strip()] = value.strip()
        
        return metadata
    
    def update_task_status(self, task_file: Path, status_update: Dict[str, Any]) -> bool:
        """
        Update task file with status information
        
        Args:
            task_file: Path to task file
            status_update: Dictionary with status updates
            
        Returns:
            True if successful
        """
        if not task_file.exists():
            return False
        
        try:
            content = task_file.read_text()
            
            # Add status update to end of file
            update_section = f"\n\n## Status Update - {datetime.now().isoformat()}\n"
            for key, value in status_update.items():
                update_section += f"- **{key}**: {value}\n"
            
            content += update_section
            task_file.write_text(content)
            
            logger.debug(f"Updated task {task_file.name} with status")
            return True
            
        except Exception as e:
            logger.error(f"Failed to update task status: {e}")
            return False
    
    def cleanup_stale_claims(self, max_age_hours: int = 24) -> int:
        """
        Clean up stale claims (tasks claimed but not completed)
        
        Args:
            max_age_hours: Maximum age in hours before considering stale
            
        Returns:
            Number of tasks cleaned up
        """
        cleaned = 0
        
        for task_file in self.in_progress_folder.glob('*.md'):
            try:
                # Get file modification time
                mtime = datetime.fromtimestamp(task_file.stat().st_mtime)
                age_hours = (datetime.now() - mtime).total_seconds() / 3600
                
                if age_hours > max_age_hours:
                    # Move back to Needs_Action
                    target = self.needs_action_folder / task_file.name
                    task_file.rename(target)
                    logger.warning(f"Cleaned up stale claim: {task_file.name} (age: {age_hours:.1f}h)")
                    cleaned += 1
                    
            except Exception as e:
                logger.error(f"Failed to cleanup stale claim {task_file}: {e}")
        
        return cleaned


class ClaimCoordinator:
    """
    Coordinator for multiple claim managers
    """
    
    def __init__(self, vault_path: str):
        """
        Initialize claim coordinator
        
        Args:
            vault_path: Path to vault directory
        """
        self.vault_path = Path(vault_path)
        self.agents: Dict[str, ClaimManager] = {}
    
    def register_agent(self, agent_name: str) -> ClaimManager:
        """
        Register an agent and return its claim manager
        
        Args:
            agent_name: Name of the agent
            
        Returns:
            ClaimManager instance for the agent
        """
        if agent_name not in self.agents:
            self.agents[agent_name] = ClaimManager(str(self.vault_path), agent_name)
            logger.info(f"Registered agent: {agent_name}")
        
        return self.agents[agent_name]
    
    def get_all_claimed_tasks(self) -> Dict[str, List[Path]]:
        """
        Get all claimed tasks grouped by agent
        
        Returns:
            Dictionary mapping agent names to their claimed tasks
        """
        claimed = {}
        
        in_progress_root = self.vault_path / 'In_Progress'
        if in_progress_root.exists():
            for agent_folder in in_progress_root.iterdir():
                if agent_folder.is_dir():
                    agent_name = agent_folder.name
                    claimed[agent_name] = list(agent_folder.glob('*.md'))
        
        return claimed
    
    def get_system_status(self) -> Dict[str, Any]:
        """
        Get overall system status
        
        Returns:
            Dictionary with system status
        """
        status = {
            'total_agents': len(self.agents),
            'agents': {},
        }
        
        for agent_name, manager in self.agents.items():
            claimed_tasks = manager.get_claimed_tasks()
            status['agents'][agent_name] = {
                'claimed_tasks': len(claimed_tasks),
                'task_files': [t.name for t in claimed_tasks],
            }
        
        return status


if __name__ == '__main__':
    # Example usage
    logging.basicConfig(level=logging.INFO)
    
    vault_path = './vault'
    
    # Create claim managers for different agents
    cloud_manager = ClaimManager(vault_path, 'cloud_agent')
    local_manager = ClaimManager(vault_path, 'local_agent')
    
    # Get available tasks
    available = cloud_manager.get_available_tasks()
    print(f"Available tasks: {len(available)}")
    
    # Claim a task
    if available:
        task = available[0]
        if cloud_manager.claim_task(task):
            print(f"Claimed: {task.name}")
