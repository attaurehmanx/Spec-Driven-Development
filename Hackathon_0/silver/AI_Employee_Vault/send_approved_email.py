"""
Email Sender for AI Employee - Silver Tier
Monitors Approved folder and sends emails via Gmail API
"""
import os
import json
import base64
from pathlib import Path
from datetime import datetime
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import time
import re

try:
    from google.oauth2.credentials import Credentials
    from google_auth_oauthlib.flow import InstalledAppFlow
    from google.auth.transport.requests import Request
    from googleapiclient.discovery import build
    GMAIL_AVAILABLE = True
except ImportError:
    GMAIL_AVAILABLE = False
    print("Warning: Google API libraries not installed.")

# Gmail API scopes - includes send permission
SCOPES = [
    'https://www.googleapis.com/auth/gmail.readonly',
    'https://www.googleapis.com/auth/gmail.send'
]

class EmailSender:
    """Email sender implementation"""

    def __init__(self, vault_path: str, credentials_path: str = 'credentials.json'):
        """
        Initialize the Email Sender

        Args:
            vault_path: Path to the Obsidian vault
            credentials_path: Path to Gmail API credentials
        """
        self.vault_path = Path(vault_path)
        self.approved_folder = self.vault_path / 'Approved'
        self.done_folder = self.vault_path / 'Done'
        self.logs_folder = self.vault_path / 'Logs'
        self.credentials_path = credentials_path
        self.token_path = 'token_sender.json'  # Separate token for sending
        self.service = None

        if not GMAIL_AVAILABLE:
            print("Gmail API libraries not available")
            return

        # Initialize Gmail service
        self._init_service()

    def _init_service(self):
        """Initialize Gmail API service with send permissions"""
        creds = None

        # Load token if exists
        if os.path.exists(self.token_path):
            creds = Credentials.from_authorized_user_file(self.token_path, SCOPES)

        # If no valid credentials, authenticate
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                try:
                    creds.refresh(Request())
                except Exception as e:
                    print(f"Token refresh failed: {e}")
                    print("Re-authenticating...")
                    creds = None

            if not creds:
                if not os.path.exists(self.credentials_path):
                    print(f"Credentials file not found: {self.credentials_path}")
                    print("Please download credentials.json from Google Cloud Console")
                    return

                flow = InstalledAppFlow.from_client_secrets_file(
                    self.credentials_path, SCOPES)
                creds = flow.run_local_server(port=0)

            # Save credentials
            with open(self.token_path, 'w') as token:
                token.write(creds.to_json())

        self.service = build('gmail', 'v1', credentials=creds)

    def parse_email_file(self, filepath: Path) -> dict:
        """Parse email approval file to extract details"""
        content = filepath.read_text(encoding='utf-8')

        # Extract YAML frontmatter
        yaml_match = re.search(r'---\n(.*?)\n---', content, re.DOTALL)
        metadata = {}
        if yaml_match:
            yaml_content = yaml_match.group(1)
            for line in yaml_content.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    metadata[key.strip()] = value.strip()

        # Extract email details from content
        to_match = re.search(r'\*\*To\*\*:\s*(.+)', content)
        subject_match = re.search(r'\*\*Subject\*\*:\s*(.+)', content)

        # Extract email address from "Name <email@example.com>" format
        to_email = None
        if to_match:
            to_text = to_match.group(1).strip()
            # Try to extract email from angle brackets
            email_match = re.search(r'<(.+?)>', to_text)
            if email_match:
                to_email = email_match.group(1)
            else:
                # If no angle brackets, assume the whole thing is an email
                to_email = to_text

        # Extract email body (everything after "### Draft Email", "### Email Body", or "### Proposed Email Body")
        body_match = re.search(r'### (?:Draft Email|Email Body|Proposed Email Body)\n\n(.*?)(?:\n---|\n##|$)', content, re.DOTALL)

        email_data = {
            'to': to_email,
            'subject': subject_match.group(1).strip() if subject_match else None,
            'body': body_match.group(1).strip() if body_match else None,
            'metadata': metadata,
            'original_file': filepath.name
        }

        return email_data

    def send_email(self, email_data: dict) -> bool:
        """Send email via Gmail API"""
        if not self.service:
            print("[ERROR] Gmail service not initialized")
            return False

        try:
            # Create message
            message = MIMEMultipart()
            message['to'] = email_data['to']
            message['subject'] = email_data['subject']

            # Add body
            body_part = MIMEText(email_data['body'], 'plain')
            message.attach(body_part)

            # Encode message
            raw_message = base64.urlsafe_b64encode(message.as_bytes()).decode('utf-8')

            # Send message
            send_message = self.service.users().messages().send(
                userId='me',
                body={'raw': raw_message}
            ).execute()

            print(f"[SUCCESS] Email sent! Message ID: {send_message['id']}")
            return True

        except Exception as e:
            print(f"[ERROR] Error sending email: {e}")
            return False

    def log_action(self, action_type: str, details: dict, result: str):
        """Log action to daily log file"""
        today = datetime.now().strftime('%Y-%m-%d')
        log_file = self.logs_folder / f'{today}.json'

        # Load existing logs
        logs = []
        if log_file.exists():
            try:
                logs = json.loads(log_file.read_text(encoding='utf-8'))
            except:
                logs = []

        # Add new log entry
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'action_type': action_type,
            'actor': 'EmailSender',
            'details': details,
            'result': result
        }
        logs.append(log_entry)

        # Save logs
        log_file.write_text(json.dumps(logs, indent=2), encoding='utf-8')

    def process_approved_emails(self):
        """Process all emails in Approved folder and subfolders"""
        if not self.approved_folder.exists():
            return

        sent_count = 0

        # Search in subfolders (gmail, whatsapp, linkedin, filesystem)
        subfolders = ['gmail', 'whatsapp', 'linkedin', 'filesystem']

        for subfolder in subfolders:
            subfolder_path = self.approved_folder / subfolder
            if not subfolder_path.exists():
                continue

            # Find all approval email files (APPROVAL_EMAIL_*.md or APPROVAL_*.md)
            email_files = list(subfolder_path.glob('APPROVAL_EMAIL_*.md'))
            email_files.extend(list(subfolder_path.glob('APPROVAL_*.md')))

            for email_file in email_files:
                try:
                    # Parse email file
                    email_data = self.parse_email_file(email_file)

                    if not email_data['to'] or not email_data['subject'] or not email_data['body']:
                        print(f"[WARNING] Skipping {email_file.name} - missing required fields")
                        continue

                    print(f"[INFO] Sending email to {email_data['to']}: {email_data['subject']}")

                    # Send email
                    success = self.send_email(email_data)

                    if success:
                        # Move to Done folder (preserve subfolder structure)
                        done_subfolder = self.done_folder / subfolder
                        done_subfolder.mkdir(parents=True, exist_ok=True)
                        done_path = done_subfolder / email_file.name
                        email_file.rename(done_path)
                        sent_count += 1

                        # Log success
                        self.log_action(
                            'email_sent',
                            {
                                'to': email_data['to'],
                                'subject': email_data['subject'],
                                'file': email_file.name,
                                'subfolder': subfolder
                            },
                            'success'
                        )

                except Exception as e:
                    print(f"[ERROR] Failed to process {email_file.name}: {e}")
                    # Log failure
                    self.log_action(
                        'email_send_failed',
                        {
                            'file': email_file.name,
                            'error': str(e)
                        },
                        'failed'
                    )

        if sent_count > 0:
            print(f"[EMAIL_SENDER] Sent {sent_count} email(s)")

    def watch_approved_folder(self, check_interval: int = 30):
        """Continuously watch Approved folder for new emails"""
        import sys
        quiet_mode = '--quiet' in sys.argv

        if not quiet_mode:
            print(f"\n{'='*60}")
            print(f"Email Sender Starting")
            print(f"{'='*60}")
            print(f"Vault Path: {self.vault_path}")
            print(f"Approved Folder: {self.approved_folder}")
            print(f"Check Interval: {check_interval} seconds")
            print(f"\nWatching for approved emails... (Press Ctrl+C to stop)")
            print(f"{'='*60}\n")
        else:
            print(f"[EMAIL_SENDER] Monitoring: {self.vault_path}")

        try:
            while True:
                self.process_approved_emails()
                time.sleep(check_interval)
        except KeyboardInterrupt:
            if not quiet_mode:
                print("\n\nEmail sender stopped by user")


if __name__ == '__main__':
    import sys

    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'
    credentials_path = sys.argv[2] if len(sys.argv) > 2 else 'credentials.json'

    # Check if running in watch mode or one-time mode
    mode = sys.argv[3] if len(sys.argv) > 3 else 'once'

    sender = EmailSender(vault_path, credentials_path)

    if mode == 'watch':
        sender.watch_approved_folder()
    else:
        sender.process_approved_emails()
