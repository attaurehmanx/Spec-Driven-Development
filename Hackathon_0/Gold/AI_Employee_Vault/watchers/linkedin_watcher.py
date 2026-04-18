"""
LinkedIn Watcher for AI Employee - Silver Tier
Monitors LinkedIn for engagement opportunities and creates posting tasks
"""
from pathlib import Path
from datetime import datetime, timedelta
from base_watcher import BaseWatcher
import json

class LinkedInWatcher(BaseWatcher):
    """LinkedIn watcher implementation"""

    def __init__(self, vault_path: str, check_interval: int = 60):
        """
        Initialize the LinkedIn watcher

        Args:
            vault_path: Path to the Obsidian vault
            check_interval: Seconds between checks (default: 60 = 1 minute)
        """
        super().__init__(vault_path, check_interval)
        self.linkedin_drafts = self.vault_path / 'LinkedIn_Drafts'
        self.linkedin_drafts.mkdir(parents=True, exist_ok=True)

        # Create linkedin subfolder in Needs_Action
        self.needs_action_linkedin = self.needs_action / 'linkedin'
        self.needs_action_linkedin.mkdir(parents=True, exist_ok=True)

        # Track last post time
        self.state_file = self.vault_path / '.linkedin_state.json'
        self.load_state()

    def load_state(self):
        """Load watcher state"""
        if self.state_file.exists():
            try:
                state = json.loads(self.state_file.read_text())
                self.last_post_time = datetime.fromisoformat(state.get('last_post_time', '2000-01-01'))
            except:
                self.last_post_time = datetime(2000, 1, 1)
        else:
            self.last_post_time = datetime(2000, 1, 1)

    def save_state(self):
        """Save watcher state"""
        state = {
            'last_post_time': self.last_post_time.isoformat()
        }
        self.state_file.write_text(json.dumps(state, indent=2))

    def check_for_updates(self) -> list:
        """Check if it's time to create a LinkedIn post"""
        now = datetime.now()

        # Check if 24 hours have passed since last post
        hours_since_last = (now - self.last_post_time).total_seconds() / 3600

        if hours_since_last >= 24:
            self.logger.info(f"Time to create LinkedIn post (last post: {hours_since_last:.1f} hours ago)")
            return [{'type': 'daily_post', 'timestamp': now.isoformat()}]
        else:
            self.logger.debug(f"Not time yet (last post: {hours_since_last:.1f} hours ago)")
            return []

    def create_action_file(self, item) -> Path:
        """Create action file for LinkedIn posting task"""
        content = f"""---
type: linkedin_post
created: {datetime.now().isoformat()}
status: pending
priority: medium
requires_approval: true
---

## LinkedIn Post Task

**Created**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
**Type**: Daily Business Post
**Status**: Awaiting Draft

## Instructions

Create a LinkedIn post about your business to generate sales and engagement.

### Post Guidelines
1. **Length**: 150-300 words
2. **Tone**: Professional but conversational
3. **Include**:
   - Value proposition or insight
   - Call to action
   - Relevant hashtags (3-5)
4. **Topics to consider**:
   - Recent project success
   - Industry insights
   - Problem-solving tips
   - Behind-the-scenes
   - Client testimonials

### Example Structure
```
[Hook - attention-grabbing first line]

[Value/Story - 2-3 paragraphs of content]

[Call to Action - what do you want readers to do?]

#hashtag1 #hashtag2 #hashtag3
```

## Next Steps
1. Draft post content below
2. Review and refine
3. Move to Pending_Approval/ for final review
4. After approval, post manually to LinkedIn
5. Move to Done/ when posted

## Draft Area
*Write your LinkedIn post draft here:*


---

## Posting Checklist
- [ ] Draft written
- [ ] Reviewed for clarity and value
- [ ] Hashtags added
- [ ] Call to action included
- [ ] Moved to Pending_Approval/
- [ ] Approved by human
- [ ] Posted to LinkedIn
- [ ] Engagement tracked
"""

        filepath = self.needs_action_linkedin / f'LINKEDIN_POST_{datetime.now().strftime("%Y%m%d")}.md'
        filepath.write_text(content, encoding='utf-8')

        # Update last post time
        self.last_post_time = datetime.now()
        self.save_state()

        return filepath


if __name__ == '__main__':
    import sys

    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'

    # Check if running in quiet mode (from orchestrator)
    quiet_mode = '--quiet' in sys.argv

    if not quiet_mode:
        print(f"\n{'='*60}")
        print(f"LinkedIn Watcher Starting")
        print(f"{'='*60}")
        print(f"Vault Path: {vault_path}")
        print(f"\nMonitoring for daily posting schedule... (Press Ctrl+C to stop)")
        print(f"{'='*60}\n")
    else:
        print(f"[LINKEDIN] Monitoring: {vault_path}")

    watcher = LinkedInWatcher(vault_path)
    watcher.run()
