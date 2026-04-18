"""
Gmail Watcher - Gold Tier Implementation
Monitors Gmail for important emails and creates action items
"""
import os
import pickle
from pathlib import Path
from datetime import datetime
from base_watcher import BaseWatcher

class GmailWatcher(BaseWatcher):
    """
    Monitors Gmail for important emails using Gmail API.

    Requires:
    - Gmail API credentials (credentials.json)
    - OAuth token (token.pickle)
    """

    def __init__(self, vault_path: str, check_interval: int = 60):
        """
        Args:
            vault_path: Path to Obsidian vault
            check_interval: Check every 1 minute (60 seconds)
        """
        super().__init__(vault_path, check_interval)

        self.gmail_folder = self.needs_action / 'email'
        self.gmail_folder.mkdir(parents=True, exist_ok=True)

        self.credentials_path = Path(__file__).parent.parent / 'credentials.json'
        self.token_path = Path(__file__).parent.parent / 'token.pickle'

        self.service = None
        self._authenticate()

    def _authenticate(self):
        """Authenticate with Gmail API"""
        try:
            from google.auth.transport.requests import Request
            from google.oauth2.credentials import Credentials
            from google_auth_oauthlib.flow import InstalledAppFlow
            from googleapiclient.discovery import build

            SCOPES = ['https://www.googleapis.com/auth/gmail.readonly']

            creds = None
            if self.token_path.exists():
                with open(self.token_path, 'rb') as token:
                    creds = pickle.load(token)

            if not creds or not creds.valid:
                if creds and creds.expired and creds.refresh_token:
                    creds.refresh(Request())
                else:
                    if not self.credentials_path.exists():
                        self.logger.warning('Gmail credentials not found - skipping authentication')
                        return

                    flow = InstalledAppFlow.from_client_secrets_file(
                        str(self.credentials_path), SCOPES
                    )
                    creds = flow.run_local_server(port=0)

                with open(self.token_path, 'wb') as token:
                    pickle.dump(creds, token)

            self.service = build('gmail', 'v1', credentials=creds)
            self.logger.info('Gmail authentication successful')

        except Exception as e:
            self.logger.error(f'Gmail authentication failed: {e}')
            self.service = None

    def check_for_updates(self) -> list:
        """Check Gmail for new important emails"""
        if not self.service:
            self.logger.info('Gmail service not available - skipping check')
            return []

        try:
            # Query for unread important emails
            results = self.service.users().messages().list(
                userId='me',
                q='is:unread is:important',
                maxResults=10
            ).execute()

            messages = results.get('messages', [])

            # Filter out already processed
            new_messages = [
                msg for msg in messages
                if msg['id'] not in self.processed_ids
            ]

            if new_messages:
                self.logger.info(f'Found {len(new_messages)} new important emails')

            return new_messages

        except Exception as e:
            self.logger.error(f'Error checking Gmail: {e}')
            return []

    def create_action_file(self, item) -> Path:
        """Create action file for email"""
        try:
            msg_id = item['id']

            # Get full message details
            message = self.service.users().messages().get(
                userId='me',
                id=msg_id,
                format='full'
            ).execute()

            # Extract headers
            headers = {
                h['name']: h['value']
                for h in message['payload']['headers']
            }

            from_email = headers.get('From', 'Unknown')
            subject = headers.get('Subject', 'No Subject')
            date = headers.get('Date', datetime.now().isoformat())

            # Extract body (simplified)
            snippet = message.get('snippet', '')

            # Calculate priority
            priority = self._calculate_priority(subject, snippet, from_email)

            # Create metadata
            metadata = {
                'from': from_email,
                'subject': subject,
                'date': date,
                'priority': priority,
                'message_id': msg_id
            }

            # Create content
            content = self.create_metadata_header('email', metadata)
            content += f"## Email from {from_email}\n\n"
            content += f"**Subject**: {subject}\n"
            content += f"**Date**: {date}\n"
            content += f"**Priority**: {priority}\n\n"
            content += f"### Preview\n{snippet}\n\n"
            content += f"### Suggested Actions\n"
            content += f"- [ ] Read full email\n"
            content += f"- [ ] Draft reply\n"
            content += f"- [ ] Forward if needed\n"
            content += f"- [ ] Archive after processing\n"

            # Create file
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            safe_subject = "".join(c for c in subject if c.isalnum() or c in (' ', '-', '_'))[:50]
            filename = f'EMAIL_{timestamp}_{safe_subject}.md'
            filepath = self.gmail_folder / filename

            filepath.write_text(content, encoding='utf-8')
            self.processed_ids.add(msg_id)

            return filepath

        except Exception as e:
            self.logger.error(f'Error creating action file for email: {e}')
            return None

    def _calculate_priority(self, subject: str, snippet: str, from_email: str) -> str:
        """Calculate email priority"""
        text = f"{subject} {snippet}".lower()

        # High priority keywords
        urgent_keywords = ['urgent', 'asap', 'emergency', 'critical', 'important']
        if any(keyword in text for keyword in urgent_keywords):
            return 'high'

        # Business keywords
        business_keywords = ['invoice', 'payment', 'contract', 'proposal', 'meeting']
        if any(keyword in text for keyword in business_keywords):
            return 'medium'

        return 'low'


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage: python gmail_watcher.py <vault_path>")
        sys.exit(1)

    vault_path = sys.argv[1]
    watcher = GmailWatcher(vault_path)
    watcher.run()
