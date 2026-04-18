"""
Gmail Watcher for AI Employee - Silver Tier
Monitors Gmail for important unread emails and creates action items
"""
import os
from pathlib import Path
from datetime import datetime
from base_watcher import BaseWatcher
import base64

try:
    from google.oauth2.credentials import Credentials
    from google_auth_oauthlib.flow import InstalledAppFlow
    from google.auth.transport.requests import Request
    from googleapiclient.discovery import build
    GMAIL_AVAILABLE = True
except ImportError:
    GMAIL_AVAILABLE = False
    print("Warning: Google API libraries not installed. Run: pip install google-auth google-auth-oauthlib google-auth-httplib2 google-api-python-client")

SCOPES = ['https://www.googleapis.com/auth/gmail.readonly']

class GmailWatcher(BaseWatcher):
    """Gmail watcher implementation"""

    def __init__(self, vault_path: str, credentials_path: str = 'credentials.json', check_interval: int = 120):
        """
        Initialize the Gmail watcher

        Args:
            vault_path: Path to the Obsidian vault
            credentials_path: Path to Gmail API credentials
            check_interval: Seconds between checks (default: 120)
        """
        super().__init__(vault_path, check_interval, watcher_type='gmail')
        self.credentials_path = credentials_path
        self.token_path = 'token.json'
        self.processed_ids = set()
        self.service = None

        if not GMAIL_AVAILABLE:
            self.logger.error("Gmail API libraries not available")
            return

        # Initialize Gmail service
        self._init_service()

    def _init_service(self):
        """Initialize Gmail API service"""
        creds = None

        # Load token if exists
        if os.path.exists(self.token_path):
            creds = Credentials.from_authorized_user_file(self.token_path, SCOPES)

        # If no valid credentials, authenticate
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                if not os.path.exists(self.credentials_path):
                    self.logger.error(f"Credentials file not found: {self.credentials_path}")
                    self.logger.info("Please download credentials.json from Google Cloud Console")
                    return

                flow = InstalledAppFlow.from_client_secrets_file(
                    self.credentials_path, SCOPES)
                creds = flow.run_local_server(port=0)

            # Save credentials
            with open(self.token_path, 'w') as token:
                token.write(creds.to_json())

        self.service = build('gmail', 'v1', credentials=creds)
        self.logger.info("Gmail service initialized successfully")

    def check_for_updates(self) -> list:
        """Check for new unread emails"""
        if not self.service:
            return []

        try:
            # Query for unread emails
            results = self.service.users().messages().list(
                userId='me',
                q='is:unread',
                maxResults=10
            ).execute()

            messages = results.get('messages', [])

            # Filter out already processed messages
            new_messages = [m for m in messages if m['id'] not in self.processed_ids]

            return new_messages

        except Exception as e:
            self.logger.error(f"Error checking Gmail: {e}")
            return []

    def create_action_file(self, message) -> Path:
        """Create action file for email"""
        try:
            # Get full message details
            msg = self.service.users().messages().get(
                userId='me',
                id=message['id'],
                format='full'
            ).execute()

            # Extract headers
            headers = {h['name']: h['value'] for h in msg['payload']['headers']}
            subject = headers.get('Subject', 'No Subject')
            sender = headers.get('From', 'Unknown')
            date = headers.get('Date', '')

            # Extract body
            body = self._get_message_body(msg)

            # Create action file
            content = f"""---
type: email
from: {sender}
subject: {subject}
received: {datetime.now().isoformat()}
date: {date}
priority: high
status: pending
message_id: {message['id']}
---

## Email Content

**From**: {sender}
**Subject**: {subject}
**Date**: {date}

### Message Body
{body[:500]}{'...' if len(body) > 500 else ''}

## Suggested Actions
- [ ] Read full email
- [ ] Reply to sender
- [ ] Forward to relevant party
- [ ] Archive after processing

## Notes
*Add response notes here*
"""

            filepath = self.needs_action / f'EMAIL_{message["id"][:8]}.md'
            filepath.write_text(content, encoding='utf-8')

            # Mark as processed
            self.processed_ids.add(message['id'])

            return filepath

        except Exception as e:
            self.logger.error(f"Error creating action file: {e}")
            raise

    def _get_message_body(self, msg):
        """Extract message body from Gmail message"""
        try:
            if 'parts' in msg['payload']:
                parts = msg['payload']['parts']
                for part in parts:
                    if part['mimeType'] == 'text/plain':
                        data = part['body'].get('data', '')
                        if data:
                            return base64.urlsafe_b64decode(data).decode('utf-8')
            else:
                data = msg['payload']['body'].get('data', '')
                if data:
                    return base64.urlsafe_b64decode(data).decode('utf-8')
        except Exception as e:
            self.logger.error(f"Error extracting body: {e}")

        return "Could not extract message body"


if __name__ == '__main__':
    import sys

    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'
    credentials_path = sys.argv[2] if len(sys.argv) > 2 else 'credentials.json'

    # Check if running in quiet mode (from orchestrator)
    quiet_mode = '--quiet' in sys.argv

    if not quiet_mode:
        print(f"\n{'='*60}")
        print(f"Gmail Watcher Starting")
        print(f"{'='*60}")
        print(f"Vault Path: {vault_path}")
        print(f"Credentials: {credentials_path}")
        print(f"\nWatching for important emails... (Press Ctrl+C to stop)")
        print(f"{'='*60}\n")
    else:
        print(f"[GMAIL] Monitoring: {vault_path}")

    watcher = GmailWatcher(vault_path, credentials_path)
    watcher.run()
