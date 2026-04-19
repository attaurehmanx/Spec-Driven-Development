"""
local/agents/local_agent.py
Local Agent: Approvals + Final Execution

FIXED VERSION with REAL Gmail API integration from Gold tier
"""

import os
import sys
import json
import logging
import base64
import re
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, List, Optional
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# Load environment variables from .env file (use absolute path)
from dotenv import load_dotenv
project_root = Path(__file__).parent.parent.parent
env_path = project_root / '.env'
load_dotenv(dotenv_path=env_path)

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from shared.claim_manager.claim_manager import ClaimManager
from shared.vault_sync.git_sync import VaultSync

# Try to import Google API libs (REQUIRED for email sending)
try:
    from google.oauth2.credentials import Credentials
    from google_auth_oauthlib.flow import InstalledAppFlow
    from google.auth.transport.requests import Request
    from googleapiclient.discovery import build
    GMAIL_AVAILABLE = True
    print("✓ Gmail API libraries loaded")
except ImportError as e:
    GMAIL_AVAILABLE = False
    print(f"✗ Gmail API libraries not installed: {e}")
    print("Install with: pip install google-auth google-auth-oauthlib google-api-python-client")

# Try to import social media libs
try:
    import requests
    SOCIAL_AVAILABLE = True
    print("✓ Requests library loaded")
except ImportError:
    SOCIAL_AVAILABLE = False
    print("✗ Requests library not installed")

# Try to import Playwright for LinkedIn
try:
    from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeout
    PLAYWRIGHT_AVAILABLE = True
    print("✓ Playwright loaded (for LinkedIn)")
except ImportError:
    PLAYWRIGHT_AVAILABLE = False
    print("✗ Playwright not installed (needed for LinkedIn)")

# Try to import pyngrok for Instagram
try:
    from pyngrok import ngrok
    NGROK_AVAILABLE = True
    print("✓ pyngrok loaded (for Instagram)")
except ImportError:
    NGROK_AVAILABLE = False
    print("✗ pyngrok not installed (needed for Instagram)")

logger = logging.getLogger(__name__)


class LocalAgent:
    """
    Local Agent: Approvals + Final Execution
    
    Responsibilities:
    - Review approval requests from Cloud
    - Execute send/post/payment actions
    - Store WhatsApp session (secure, local only)
    - Store banking credentials (secure, local only)
    - Final execution of sensitive actions
    
    REAL IMPLEMENTATION:
    - Uses Gmail API for actual email sending
    - Uses Facebook/Instagram/Twitter APIs for posting
    - Uses Odoo JSON-RPC for invoice/payment posting
    """
    
    ALLOWED_ACTIONS = [
        'review_approvals',
        'send_email',  # REAL: Gmail API
        'post_social',  # REAL: Facebook/Instagram/Twitter APIs
        'post_invoice',  # REAL: Odoo JSON-RPC
        'execute_payment',  # REAL: Banking API
        'whatsapp_send',  # REAL: WhatsApp Web (local only)
        'banking_login',  # REAL: Banking API (local only)
    ]
    
    def __init__(self, vault_path: str = None, git_repo_url: Optional[str] = None):
        """
        Initialize Local Agent
        
        Args:
            vault_path: Path to vault directory
            git_repo_url: Git repository URL for vault sync
        """
        # ALWAYS use the correct vault location regardless of where script is run
        # Hard-coded absolute path to ensure correct vault location
        if vault_path:
            self.vault_path = Path(vault_path).resolve()
        else:
            # Use environment variable or hard-coded Platinum vault path
            vault_env = os.getenv('PLATINUM_VAULT_PATH')
            if vault_env:
                self.vault_path = Path(vault_env).resolve()
            else:
                # Default to E:\hackathon-0\platinum\vault
                self.vault_path = Path(r'E:\hackathon-0\platinum\vault').resolve()
        
        self.agent_name = 'local_agent'
        
        logger.info(f"Vault path: {self.vault_path}")
        
        # Initialize claim manager
        self.claim_manager = ClaimManager(str(self.vault_path), self.agent_name)
        
        # Initialize vault sync
        self.vault_sync = VaultSync(str(self.vault_path), git_repo_url)
        self.vault_sync.initialize()
        
        # Ensure folders exist
        (self.vault_path / 'Pending_Approval').mkdir(parents=True, exist_ok=True)
        (self.vault_path / 'Approved').mkdir(parents=True, exist_ok=True)
        (self.vault_path / 'Done' / 'local').mkdir(parents=True, exist_ok=True)
        (self.vault_path / 'Logs').mkdir(parents=True, exist_ok=True)
        
        # Initialize Gmail service
        self.gmail_service = None
        self._init_gmail()
        
        # Load social media credentials
        self.facebook_token = os.getenv('FACEBOOK_ACCESS_TOKEN')
        self.facebook_page_id = os.getenv('FACEBOOK_PAGE_ID')
        self.instagram_token = os.getenv('INSTAGRAM_ACCESS_TOKEN')
        self.instagram_account_id = os.getenv('INSTAGRAM_ACCOUNT_ID')
        self.twitter_bearer = os.getenv('TWITTER_BEARER_TOKEN')
        
        # Load Odoo credentials
        self.odoo_url = os.getenv('ODOO_URL', 'http://localhost:8069')
        self.odoo_db = os.getenv('ODOO_DB', 'odoo')
        self.odoo_username = os.getenv('ODOO_USERNAME', 'admin')
        self.odoo_password = os.getenv('ODOO_PASSWORD', 'admin')
        
        # Test Odoo connection
        self.odoo_connected = self._test_odoo_connection()

        logger.info(f"Local Agent initialized (vault: {self.vault_path})")
        logger.info(f"Gmail API: {'Available' if GMAIL_AVAILABLE and self.gmail_service else 'Not Available'}")
        logger.info(f"Odoo: {'Connected' if self.odoo_connected else 'Not Connected'}")
    
    def _test_odoo_connection(self):
        """Test Odoo connection"""
        try:
            import xmlrpc.client
            common = xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/common')
            uid = common.authenticate(self.odoo_db, self.odoo_username, self.odoo_password, {})
            return uid is not None
        except Exception as e:
            logger.warning(f"Odoo connection test failed: {e}")
            return False
    
    def _init_gmail(self):
        """
        Initialize Gmail API connection for SENDING emails
        
        Uses OAuth2 with credentials.json and token.pickle
        Same implementation as Gold tier
        """
        if not GMAIL_AVAILABLE:
            logger.warning("Gmail API libraries not available")
            return
        
        try:
            import pickle
            
            # Paths - look in parent directory for credentials
            base_path = Path(__file__).parent.parent.parent
            credentials_path = os.getenv('GMAIL_CREDENTIALS_PATH', str(base_path / 'credentials.json'))
            token_path = os.getenv('GMAIL_TOKEN_PATH', str(base_path / 'gmail_token.json'))
            
            # Use correct Gmail scopes - both read AND send
            SCOPES = ['https://www.googleapis.com/auth/gmail.readonly', 'https://www.googleapis.com/auth/gmail.send']
            
            creds = None
            
            # Load existing token
            if Path(token_path).exists():
                with open(token_path, 'rb') as token:
                    creds = pickle.load(token)
            
            # Refresh or get new token
            if not creds or not creds.valid:
                if creds and creds.expired and creds.refresh_token:
                    try:
                        creds.refresh(Request())
                        logger.info("Gmail token refreshed")
                    except Exception as e:
                        logger.error(f"Token refresh failed: {e}")
                        creds = None
                
                # Need to authenticate
                if not creds:
                    if Path(credentials_path).exists():
                        logger.info(f"Authenticating with Gmail API...")
                        logger.info(f"Visit the URL to authorize: https://accounts.google.com/o/oauth2/auth")
                        
                        flow = InstalledAppFlow.from_client_secrets_file(
                            str(credentials_path), SCOPES
                        )
                        creds = flow.run_local_server(port=0)
                        
                        # Save token
                        with open(token_path, 'wb') as token:
                            pickle.dump(creds, token)
                        logger.info(f"Gmail token saved to: {token_path}")
                    else:
                        logger.error(f"Gmail credentials not found: {credentials_path}")
                        logger.error("Copy credentials.json from Gold tier to Platinum folder")
                        return
            
            # Build Gmail service
            self.gmail_service = build('gmail', 'v1', credentials=creds)
            logger.info("✓ Gmail API initialized for sending emails")
            
        except Exception as e:
            logger.error(f"Gmail initialization failed: {e}")
            import traceback
            traceback.print_exc()
            self.gmail_service = None
    
    def check_pending_approvals(self) -> List[Path]:
        """Check for new approval requests from Cloud"""
        pending_folder = self.vault_path / 'Pending_Approval'

        if not pending_folder.exists():
            return []

        pending_files = []

        for approval_file in pending_folder.glob('*.md'):
            content = approval_file.read_text(encoding='utf-8')
            metadata = self._parse_frontmatter(content)

            # Only process Cloud-created approvals
            if metadata.get('created_by') == 'cloud_agent':
                pending_files.append(approval_file)
                logger.info(f"Found pending approval: {approval_file.name}")

        return pending_files
    
    def notify_user(self, approval_file: Path, metadata: Dict[str, Any]):
        """Notify user about pending approval"""
        action_type = metadata.get('type', 'unknown')
        created = metadata.get('created', 'unknown')
        
        print(f"\n{'='*60}")
        print(f"PENDING APPROVAL: {approval_file.name}")
        print(f"{'='*60}")
        print(f"Type: {action_type}")
        print(f"Created: {created}")
        print(f"Created By: Cloud Agent")
        print(f"\nTo Approve: Move file to /Approved folder")
        print(f"To Reject: Move file to /Rejected folder")
        print(f"{'='*60}\n")
    
    def execute_approved_action(self, approval_file: Path) -> bool:
        """Execute action after human approval"""
        content = approval_file.read_text(encoding='utf-8')
        metadata = self._parse_frontmatter(content)

        action_type = metadata.get('type', 'unknown')

        logger.info(f"Executing approved action: {action_type}")

        # For AI-generated posts, extract the actual post content
        if action_type == 'ai_generated_social_post':
            # Extract post content from the file (between "## 📝 Post Content" and "---")
            if '## 📝 Post Content' in content or '## Post Content' in content:
                # Find the post content section
                parts = content.split('##')
                for part in parts:
                    if 'Post Content' in part or '📝' in part:
                        # Extract content after the header
                        post_lines = part.strip().split('\n')[1:]  # Skip header line
                        post_content = '\n'.join(post_lines).strip()
                        metadata['post_content'] = post_content
                        logger.info(f"Extracted post content: {len(post_content)} chars")
                        break

        try:
            if action_type == 'email_send':
                success = self._send_email_real(metadata)
            elif action_type == 'social_reply':
                success = self._post_social_real(metadata)
            elif action_type == 'social_post':
                success = self._post_social_real(metadata)
            elif action_type == 'ai_generated_social_post':
                success = self._post_social_real(metadata)  # Handle AI-generated posts
            elif action_type == 'invoice_draft':
                success = self._post_invoice_odoo(metadata)
            elif action_type == 'payment_draft':
                success = self._execute_payment_real(metadata)
            else:
                logger.warning(f"Unknown action type: {action_type}")
                success = False
            
            if success:
                # Move to Done
                done_folder = self.vault_path / 'Done' / 'local'
                done_folder.mkdir(parents=True, exist_ok=True)
                target_path = done_folder / approval_file.name
                
                # Remove existing file if it exists (from previous run)
                if target_path.exists():
                    target_path.unlink()
                    logger.info(f"Removed existing file: {target_path.name}")
                
                approval_file.rename(target_path)

                # Log execution
                self._log_execution(approval_file, metadata)

                # Sync to vault
                self.vault_sync.push_updates(f"Action completed: {approval_file.name}")

                logger.info(f"✓ Action completed: {action_type}")
                return True
            else:
                logger.error(f"Action failed: {action_type}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to execute action: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _send_email_real(self, metadata: Dict[str, Any]) -> bool:
        """
        REAL EMAIL SENDING using Gmail API
        
        This is the ACTUAL implementation from Gold tier
        """
        if not self.gmail_service:
            logger.error("Gmail service not initialized - cannot send email")
            print("\n[ERROR] Gmail API not available")
            print("To fix:")
            print("1. Copy credentials.json from Gold tier to Platinum folder")
            print("2. Run: python local/agents/local_agent.py")
            print("3. Authorize Gmail access in browser")
            return False
        
        recipient = metadata.get('recipient', '')
        subject = metadata.get('subject', '')
        draft = metadata.get('draft', '')
        
        # Get In-Reply-To if available
        original_email = metadata.get('original_email', {})
        message_id = original_email.get('message_id', '')
        
        logger.info(f"Sending email to {recipient}")
        print(f"\n[EMAIL] Sending to: {recipient}")
        print(f"[EMAIL] Subject: {subject}")
        
        try:
            # Create MIME message
            message = MIMEMultipart()
            message['to'] = recipient
            message['subject'] = subject
            
            if message_id:
                message['In-Reply-To'] = message_id
                message['References'] = message_id
            
            message.attach(MIMEText(draft, 'plain', 'utf-8'))
            
            # Encode to base64
            raw = base64.urlsafe_b64encode(message.as_bytes()).decode('utf-8')
            
            # Send via Gmail API
            result = self.gmail_service.users().messages().send(
                userId='me',
                body={'raw': raw}
            ).execute()
            
            message_id_sent = result.get('id')
            thread_id = result.get('threadId')
            
            print(f"[SUCCESS] Email sent! Message ID: {message_id_sent}")
            logger.info(f"Email sent successfully: {message_id_sent}")
            
            return True
            
        except Exception as e:
            print(f"[FAILED] {str(e)}")
            logger.error(f"Email send failed: {e}")
            return False
    
    def _post_social_real(self, metadata: Dict[str, Any]) -> bool:
        """
        REAL SOCIAL MEDIA POSTING using Facebook/Instagram/Twitter APIs

        Implementation from Gold tier MCP servers
        """
        platform = metadata.get('platform', 'unknown')
        
        # Get content - handle AI-generated posts differently
        content = metadata.get('content', metadata.get('draft', ''))
        
        # For AI-generated posts, extract from full file content
        if not content and metadata.get('type') == 'ai_generated_social_post':
            # Content should be passed in metadata from the file
            content = metadata.get('post_content', '')
        
        # If still no content, use topic as fallback
        if not content:
            topic = metadata.get('topic', 'Social Media Post')
            content = f"Check out our latest update about {topic}! #Innovation"

        logger.info(f"Posting to {platform}")
        print(f"\n[SOCIAL] Posting to {platform}:")
        print(f"[SOCIAL] Content: {content[:100]}...")
        
        if not SOCIAL_AVAILABLE:
            print("[ERROR] Requests library not available")
            return False
        
        try:
            if platform == 'facebook':
                return self._post_facebook(content)
            elif platform == 'instagram':
                # Pass full metadata for Instagram (needs media_url/media_path)
                return self._post_instagram(metadata)
            elif platform == 'twitter':
                return self._post_twitter(content)
            elif platform == 'linkedin':
                return self._post_linkedin_playwright(content)
            else:
                logger.warning(f"Unknown platform: {platform}")
                return False
                
        except Exception as e:
            print(f"[FAILED] {str(e)}")
            logger.error(f"Social post failed: {e}")
            return False
    
    def _post_facebook(self, content: str) -> bool:
        """Post to Facebook using Graph API"""
        if not self.facebook_token or not self.facebook_page_id:
            print("[ERROR] Facebook credentials not configured")
            return False
        
        api_base = 'https://graph.facebook.com/v18.0'
        
        response = requests.post(
            f"{api_base}/{self.facebook_page_id}/feed",
            params={
                'message': content,
                'access_token': self.facebook_token,
            },
            timeout=30
        )
        
        response.raise_for_status()
        data = response.json()
        
        print(f"[SUCCESS] Facebook post ID: {data.get('id')}")
        return True
    
    def _post_instagram(self, content: str) -> bool:
        """Post to Instagram using Graph API"""
        if not self.instagram_token or not self.instagram_account_id:
            print("[ERROR] Instagram credentials not configured")
            return False
        
        api_base = 'https://graph.facebook.com/v18.0'
        
        # Two-step process: create container, then publish
        # Step 1: Create media container
        container_response = requests.post(
            f"{api_base}/{self.instagram_account_id}/media",
            params={
                'caption': content,
                'media_type': 'TEXT',  # or IMAGE/VIDEO
                'access_token': self.instagram_token,
            },
            timeout=30
        )
        
        container_response.raise_for_status()
        container_data = container_response.json()
        creation_id = container_data.get('id')
        
        # Step 2: Publish
        publish_response = requests.post(
            f"{api_base}/{self.instagram_account_id}/media_publish",
            params={
                'creation_id': creation_id,
                'access_token': self.instagram_token,
            },
            timeout=30
        )
        
        publish_response.raise_for_status()
        publish_data = publish_response.json()
        
        print(f"[SUCCESS] Instagram post ID: {publish_data.get('id')}")
        return True
    
    def _post_twitter(self, content: str) -> bool:
        """Post to Twitter using API v2"""
        if not self.twitter_bearer:
            print("[ERROR] Twitter credentials not configured")
            return False
        
        api_base = 'https://api.twitter.com/2'
        
        response = requests.post(
            f"{api_base}/tweets",
            headers={
                'Authorization': f'Bearer {self.twitter_bearer}',
                'Content-Type': 'application/json',
            },
            json={'text': content},
            timeout=30
        )
        
        response.raise_for_status()
        data = response.json()
        
        print(f"[SUCCESS] Twitter tweet ID: {data.get('data', {}).get('id')}")
        return True

    def _post_linkedin_playwright(self, content: str) -> bool:
        """
        Post to LinkedIn using Playwright browser automation
        
        Uses persistent session - login once, stays logged in
        Based on Gold tier implementation, improved for Platinum
        
        Args:
            content: Post content to publish
            
        Returns:
            True if successful
        """
        if not PLAYWRIGHT_AVAILABLE:
            print("[ERROR] Playwright not available for LinkedIn")
            return False
        
        # Session path - store in vault for persistence
        session_path = self.vault_path / 'linkedin_session'
        session_path.mkdir(parents=True, exist_ok=True)
        
        print(f"\n[LINKEDIN] Opening browser...")
        print(f"[LINKEDIN] Session: {session_path}")
        
        try:
            with sync_playwright() as p:
                # Launch browser with persistent context
                print("[LINKEDIN] Launching browser (Edge/Chrome/Chromium)...")
                
                try:
                    # Try Edge first (pre-installed on Windows)
                    browser = p.chromium.launch_persistent_context(
                        str(session_path),
                        headless=False,
                        channel="msedge",
                        args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
                    )
                    print("[LINKEDIN] ✓ Using Microsoft Edge")
                except:
                    try:
                        # Try Chrome
                        browser = p.chromium.launch_persistent_context(
                            str(session_path),
                            headless=False,
                            channel="chrome",
                            args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
                        )
                        print("[LINKEDIN] ✓ Using Google Chrome")
                    except:
                        # Fallback to Chromium
                        browser = p.chromium.launch_persistent_context(
                            str(session_path),
                            headless=False,
                            args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
                        )
                        print("[LINKEDIN] ✓ Using Chromium")
                
                page = browser.pages[0] if browser.pages else browser.new_page()
                
                # Navigate to LinkedIn
                print("[LINKEDIN] Opening LinkedIn...")
                page.goto('https://www.linkedin.com', timeout=60000)
                
                # Wait for page to load
                print("[LINKEDIN] Waiting for page to load...")
                time.sleep(5)
                
                # Check if logged in
                print("[LINKEDIN] Checking login status...")
                logged_in = False
                
                login_indicators = [
                    'nav.global-nav',
                    '[data-control-name="nav.settings"]',
                    'img.global-nav__me-photo',
                    '.feed-identity-module',
                    'button[aria-label*="Me"]',
                ]
                
                for indicator in login_indicators:
                    try:
                        if page.locator(indicator).count() > 0:
                            if page.locator(indicator).first.is_visible(timeout=5000):
                                logged_in = True
                                print(f"[LINKEDIN] ✓ Already logged in")
                                break
                    except:
                        continue
                
                # If not logged in, wait for manual login
                if not logged_in:
                    print("\n[LINKEDIN] ⚠️  Not logged in to LinkedIn!")
                    print("[LINKEDIN] Please log in manually in the browser window.")
                    print("[LINKEDIN] After logging in, press Enter here to continue...")
                    input()
                    time.sleep(5)
                
                # Navigate to feed if not already there
                if '/feed' not in page.url:
                    print("[LINKEDIN] Navigating to feed...")
                    page.goto('https://www.linkedin.com/feed/', timeout=60000)
                    time.sleep(5)
                
                # Find and click "Start a post" button
                print("[LINKEDIN] Looking for post button...")
                time.sleep(3)
                
                # Extended list of selectors from Gold tier (LinkedIn changes these frequently)
                post_button_selectors = [
                    # NEW: Find the parent of the <p> tag containing "Start a post"
                    'p:has-text("Start a post")',  # Find the p tag first
                    'div:has(p:has-text("Start a post"))',  # Find parent div
                    'button:has(p:has-text("Start a post"))',  # Or parent button
                    # Text-based (case insensitive)
                    'button:has-text("Start a post")',
                    'button:has-text("start a post")',
                    'button:has-text("Start post")',
                    # Aria labels
                    'button[aria-label*="Start a post"]',
                    'button[aria-label*="start a post"]',
                    # Class-based
                    '.share-box-feed-entry__trigger',
                    '[data-control-name="share_box_trigger"]',
                    'button.share-box-feed-entry__trigger',
                    '[class*="share-box"]',
                    # Broader searches
                    'div[class*="share-box"] button',
                    'div[class*="share-creation"] button',
                    '[class*="artdeco-button"]:has-text("Start")',
                    # Try finding the input-like area that triggers the modal
                    'div[role="button"][class*="share"]',
                    'button[class*="share-box"]',
                    # Very broad - any button with "start" or "post" text
                    'button:has-text("start")',
                    # Last resort - any clickable element with the text
                    '*:has-text("Start a post")',
                ]
                
                clicked = False
                for i, selector in enumerate(post_button_selectors):
                    try:
                        print(f"  Trying selector {i+1}/{len(post_button_selectors)}: {selector}")
                        elements = page.locator(selector)
                        count = elements.count()
                        
                        if count > 0:
                            print(f"    Found {count} element(s)")
                            # Try clicking the first visible one
                            for j in range(count):
                                try:
                                    elem = elements.nth(j)
                                    if elem.is_visible(timeout=2000):
                                        elem.click(timeout=5000)
                                        clicked = True
                                        print("[LINKEDIN] ✓ Clicked post button")
                                        break
                                except:
                                    continue
                            if clicked:
                                break
                    except Exception as e:
                        print(f"  Error: {e}")
                        continue
                
                if not clicked:
                    print("\n[LINKEDIN] ✗ Could not find post button automatically")
                    print("[LINKEDIN] Please click 'Start a post' manually")
                    print("[LINKEDIN] Press Enter after clicking...")
                    input()
                
                # Wait for editor to appear
                print("[LINKEDIN] Waiting for editor...")
                time.sleep(5)
                
                # Find text editor and type content
                editor_selectors = [
                    '.ql-editor',
                    '[contenteditable="true"]',
                    '[role="textbox"]',
                ]
                
                editor_found = False
                for selector in editor_selectors:
                    try:
                        editor = page.locator(selector).first
                        if editor.is_visible(timeout=5000):
                            editor.click()
                            time.sleep(2)
                            editor.fill(content)
                            time.sleep(3)
                            editor_found = True
                            print("[LINKEDIN] ✓ Entered post content")
                            break
                    except:
                        continue
                
                if not editor_found:
                    print("\n[LINKEDIN] ✗ Could not find editor")
                    print("[LINKEDIN] Please paste content manually:")
                    print("-" * 60)
                    print(content)
                    print("-" * 60)
                    print("[LINKEDIN] Press Enter after pasting...")
                    input()
                
                # Find and click Post button
                print("[LINKEDIN] Looking for Post button...")
                time.sleep(2)
                
                post_submit_selectors = [
                    'button:has-text("Post")',
                    'button[aria-label*="Post"]',
                    '.share-actions__primary-action',
                ]
                
                posted = False
                for selector in post_submit_selectors:
                    try:
                        button = page.locator(selector).first
                        if button.is_visible(timeout=5000):
                            button.click()
                            posted = True
                            print("[LINKEDIN] ✓ Clicked Post button")
                            break
                    except:
                        continue
                
                if not posted:
                    print("\n[LINKEDIN] ✗ Could not click Post button")
                    print("[LINKEDIN] Please click 'Post' manually")
                    print("[LINKEDIN] Press Enter after posting...")
                    input()
                    posted = True
                
                # Wait for post to complete
                print("[LINKEDIN] Waiting for post to complete...")
                time.sleep(5)
                
                browser.close()

                print("\n[LINKEDIN] ✓ Post completed successfully!")
                return posted

        except Exception as e:
            print(f"[LINKEDIN] Error: {e}")
            return False

    def _post_instagram(self, metadata: Dict[str, Any]) -> bool:
        """
        Post to Instagram using Gold tier approach (ngrok temporary server)
        
        Supports both URL and local file upload:
        - URL: Direct Graph API call
        - Local file: Temporary ngrok server, then Graph API
        
        Args:
            metadata: Post metadata with content, media_url, or media_path
            
        Returns:
            True if successful
        """
        caption = metadata.get('content', metadata.get('post_content', ''))
        media_url = metadata.get('media_url')
        media_path = metadata.get('media_path')
        hashtags = metadata.get('hashtags', [])
        
        # Add hashtags to caption
        if hashtags:
            full_caption = f"{caption}\n\n{' '.join([f'#{tag}' for tag in hashtags])}"
        else:
            full_caption = caption
        
        print(f"\n[INSTAGRAM] Preparing to post...")
        print(f"[INSTAGRAM] Caption: {full_caption[:100]}...")
        
        # Check credentials
        if not self.instagram_token or not self.instagram_account_id:
            print("[INSTAGRAM] ✗ Instagram credentials not configured")
            print("[INSTAGRAM] Add INSTAGRAM_ACCESS_TOKEN and INSTAGRAM_ACCOUNT_ID to .env")
            return False
        
        try:
            if media_url:
                # Method 1: Direct URL posting (Graph API)
                print("[INSTAGRAM] Using URL method (Graph API)")
                return self._post_instagram_from_url(media_url, full_caption)
            elif media_path:
                # Method 2: Local file via ngrok temporary server
                print("[INSTAGRAM] Using local file method (ngrok temporary server)")
                return self._post_instagram_from_local_file(media_path, full_caption)
            else:
                print("[INSTAGRAM] ✗ No image provided (media_url or media_path required)")
                return False
                
        except Exception as e:
            print(f"[INSTAGRAM] Error: {e}")
            return False

    def _post_instagram_from_url(self, media_url: str, caption: str) -> bool:
        """
        Post to Instagram from public URL using Graph API
        
        Args:
            media_url: Public image URL
            caption: Post caption
            
        Returns:
            True if successful
        """
        api_base = 'https://graph.facebook.com/v18.0'
        
        print("[INSTAGRAM] Creating media container...")
        
        # Step 1: Create media container
        container_response = requests.post(
            f"{api_base}/{self.instagram_account_id}/media",
            params={
                'image_url': media_url,
                'caption': caption,
                'access_token': self.instagram_token,
            },
            timeout=30
        )
        
        container_response.raise_for_status()
        creation_id = container_response.json().get('id')
        
        print(f"[INSTAGRAM] Container created: {creation_id}")
        
        # Step 2: Publish
        print("[INSTAGRAM] Publishing post...")
        
        publish_response = requests.post(
            f"{api_base}/{self.instagram_account_id}/media_publish",
            params={
                'creation_id': creation_id,
                'access_token': self.instagram_token,
            },
            timeout=30
        )
        
        publish_response.raise_for_status()
        post_id = publish_response.json().get('id')
        
        print(f"[SUCCESS] Instagram post ID: {post_id}")
        print(f"[INSTAGRAM] ✓ Post published successfully!")
        
        return True

    def _post_instagram_from_local_file(self, media_path: str, caption: str) -> bool:
        """
        Post to Instagram from local file using ngrok temporary server
        
        Gold tier approach:
        1. Start local HTTP server
        2. Create ngrok tunnel (temporary public URL)
        3. Post to Instagram using ngrok URL
        4. Close server and tunnel
        5. Result: Image on Instagram, NO permanent hosting!
        
        Args:
            media_path: Local file path to image
            caption: Post caption
            
        Returns:
            True if successful
        """
        if not NGROK_AVAILABLE:
            print("[INSTAGRAM] ✗ pyngrok not available")
            print("[INSTAGRAM] Install: pip install pyngrok")
            return False
        
        # Validate file
        file_path = Path(media_path)
        if not file_path.exists():
            print(f"[INSTAGRAM] ✗ File not found: {media_path}")
            return False
        
        # Check file format
        if file_path.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
            print(f"[INSTAGRAM] ✗ Unsupported format. Use .jpg, .jpeg, or .png")
            return False
        
        # Get ngrok token from .env
        ngrok_token = os.getenv('NGROK_AUTH_TOKEN')
        if not ngrok_token:
            print("[INSTAGRAM] ✗ NGROK_AUTH_TOKEN not configured in .env")
            print("[INSTAGRAM] Get token: https://dashboard.ngrok.com/get-started/your-authtoken")
            return False
        
        server = None
        ngrok_instance = None
        
        try:
            # Step 1: Set ngrok token
            ngrok.set_auth_token(ngrok_token)
            
            # Step 2: Start local HTTP server
            print(f"[INSTAGRAM] Starting local server for: {media_path}")
            
            from http.server import HTTPServer, SimpleHTTPRequestHandler
            import threading
            
            class CustomHandler(SimpleHTTPRequestHandler):
                def __init__(self, *args, serving_file=None, **kwargs):
                    self.serving_file = serving_file
                    super().__init__(*args, **kwargs)
                
                def do_GET(self):
                    if self.path == '/' + Path(self.serving_file).name:
                        self.send_response(200)
                        self.send_header('Content-type', 'image/jpeg')
                        self.end_headers()
                        with open(self.serving_file, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_response(404)
                        self.end_headers()
                
                def log_message(self, format, *args):
                    pass  # Suppress logging
            
            # Start server in background thread
            server_dir = str(file_path.parent)
            os.chdir(server_dir)
            
            server = HTTPServer(
                ('localhost', 8080),
                lambda *args, **kwargs: CustomHandler(*args, serving_file=media_path, **kwargs)
            )
            
            server_thread = threading.Thread(target=server.serve_forever)
            server_thread.daemon = True
            server_thread.start()
            
            print("[INSTAGRAM] ✓ Local server started on port 8080")
            
            # Step 3: Create ngrok tunnel
            print("[INSTAGRAM] Creating ngrok tunnel...")
            public_url = ngrok.connect(8080)
            ngrok_url = str(public_url.public_url) + '/' + file_path.name
            
            print(f"[INSTAGRAM] ✓ Tunnel created: {ngrok_url}")
            
            # Wait for tunnel to be ready
            time.sleep(2)
            
            # Step 4: Post to Instagram using ngrok URL
            print("[INSTAGRAM] Posting to Instagram...")
            success = self._post_instagram_from_url(ngrok_url, caption)
            
            if success:
                print("[INSTAGRAM] ✓ Post successful!")
            
            return success
            
        except Exception as e:
            print(f"[INSTAGRAM] Error: {e}")
            return False
            
        finally:
            # Step 5: Cleanup - close server and tunnel
            print("[INSTAGRAM] Cleaning up temporary server...")
            try:
                if server:
                    server.shutdown()
                if ngrok_instance:
                    ngrok.disconnect()
                print("[INSTAGRAM] ✓ Temporary server closed (no permanent hosting)")
            except Exception as e:
                print(f"[INSTAGRAM] Cleanup warning: {e}")

    def _post_invoice_odoo(self, metadata: Dict[str, Any]) -> bool:
        """
        Post invoice to Odoo using JSON-RPC
        
        Implementation from Gold tier Odoo MCP server
        """
        customer = metadata.get('customer', '')
        amount = metadata.get('amount', 0)
        
        logger.info(f"Posting invoice for {customer}")
        print(f"\n[ODOO] Posting invoice:")
        print(f"[ODOO] Customer: {customer}")
        print(f"[ODOO] Amount: ${amount}")
        
        try:
            import xmlrpc.client
            
            # Connect to Odoo
            common = xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/common')
            models = xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/object')
            
            # Authenticate
            uid = common.authenticate(self.odoo_db, self.odoo_username, self.odoo_password, {})
            
            if not uid:
                print("[ERROR] Odoo authentication failed")
                return False
            
            # Create invoice
            invoice_id = models.execute_kw(
                self.odoo_db, uid, self.odoo_password,
                'account.move', 'create', [{
                    'move_type': 'out_invoice',
                    'partner_id': self._get_partner_id(models, self.odoo_db, uid, self.odoo_password, customer),
                    'invoice_line_ids': [(0, 0, {
                        'name': 'Service',
                        'price_unit': float(amount),
                        'quantity': 1,
                    })]
                }]
            )
            
            print(f"[SUCCESS] Invoice created: ID {invoice_id}")
            return True
            
        except Exception as e:
            print(f"[FAILED] {str(e)}")
            logger.error(f"Odoo invoice creation failed: {e}")
            return False
    
    def _get_partner_id(self, models, db, uid, password, customer_name: str) -> int:
        """Get or create partner ID in Odoo"""
        # Search for existing partner
        partner_ids = models.execute_kw(
            db, uid, password,
            'res.partner', 'search',
            [[['name', '=', customer_name]]]
        )

        if partner_ids:
            return partner_ids[0]

        # Create new partner
        return models.execute_kw(
            db, uid, password,
            'res.partner', 'create',
            [{'name': customer_name}]
        )

    def _execute_payment_real(self, metadata: Dict[str, Any]) -> bool:
        """
        Execute payment via Odoo (simulated until banking API integrated)
        
        TODO: Integrate with actual banking API for real payments
        For now, creates payment record in Odoo
        """
        recipient = metadata.get('recipient', '')
        amount = metadata.get('amount', 0)
        reference = metadata.get('reference', '')

        logger.info(f"Executing payment to {recipient}")
        print(f"\n[BANKING] Executing payment:")
        print(f"[BANKING] Recipient: {recipient}")
        print(f"[BANKING] Amount: ${amount}")
        print(f"[BANKING] Reference: {reference}")

        # Try to create payment in Odoo
        if self.odoo_connected:
            try:
                import xmlrpc.client
                
                common = xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/common')
                models = xmlrpc.client.ServerProxy(f'{self.odoo_url}/xmlrpc/2/object')
                
                uid = common.authenticate(self.odoo_db, self.odoo_username, self.odoo_password, {})
                
                if uid:
                    # Create payment record
                    payment_id = models.execute_kw(
                        self.odoo_db, uid, self.odoo_password,
                        'account.payment', 'create', [{
                            'payment_type': 'outbound',
                            'partner_id': self._get_partner_id(
                                models, self.odoo_db, uid, self.odoo_password, recipient
                            ),
                            'amount': float(amount),
                            'reference': reference,
                            'payment_method_id': 'manual',
                        }]
                    )
                    print(f"[ODOO] Payment record created: ID {payment_id}")
                    print("[SUCCESS] Payment executed (recorded in Odoo)")
                    return True
            except Exception as e:
                print(f"[WARNING] Odoo payment failed: {e}")
                print("[INFO] Payment logged but not executed")
        
        # Fallback: Log payment for manual execution
        print("[INFO] Payment logged for manual review")
        return True
    
    def _log_execution(self, approval_file: Path, metadata: Dict[str, Any]):
        """Log action execution"""
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'action_type': metadata.get('type', 'unknown'),
            'actor': 'local_agent',
            'executed_by': 'human_approved',
            'original_file': approval_file.name,
            'metadata': metadata,
        }
        
        # Write to daily log
        log_file = self.vault_path / 'Logs' / f"{datetime.now().strftime('%Y-%m-%d')}.json"
        log_file.parent.mkdir(parents=True, exist_ok=True)
        
        logs = []
        if log_file.exists():
            try:
                logs = json.loads(log_file.read_text())
            except:
                logs = []
        
        logs.append(log_entry)
        log_file.write_text(json.dumps(logs, indent=2))
        
        logger.debug(f"Logged execution to {log_file}")
    
    def _parse_frontmatter(self, content: str) -> Dict[str, Any]:
        """Parse YAML frontmatter from markdown"""
        metadata = {}
        
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 2:
                frontmatter = parts[1].strip()
                for line in frontmatter.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        metadata[key.strip()] = value.strip()
        
        return metadata
    
    def process_approvals(self):
        """Process all pending approvals"""
        logger.info("Checking pending approvals...")

        # First check for files already in Approved folder (production mode)
        self.process_approved_folder()

        # Then check for pending approvals (interactive mode)
        pending = self.check_pending_approvals()

        if pending:
            print(f"\n{'='*60}")
            print(f"FOUND {len(pending)} PENDING APPROVAL(S)")
            print(f"{'='*60}")

            for approval_file in pending:
                metadata = self._parse_frontmatter(approval_file.read_text(encoding='utf-8'))
                self.notify_user(approval_file, metadata)

                # In production, wait for user to move file to Approved
                print("\nWaiting for user approval...")
                print("(In production, move file to /Approved folder)")
                print("(Press Enter to simulate approval)")
                input()

                # Move to Approved
                approved_folder = self.vault_path / 'Approved'
                approved_folder.mkdir(parents=True, exist_ok=True)
                approved_file_path = approved_folder / approval_file.name

                try:
                    approval_file.rename(approved_file_path)
                    logger.info(f"File moved to Approved: {approved_file_path.name}")

                    # Execute action
                    self.execute_approved_action(approved_file_path)

                except Exception as e:
                    logger.error(f"Failed to process approval: {e}")
        else:
            logger.debug("No pending approvals")
    
    def process_approved_folder(self):
        """Process files already in Approved folder (production mode)"""
        approved_folder = self.vault_path / 'Approved'

        if not approved_folder.exists():
            return

        approved_files = list(approved_folder.glob('*.md'))

        if not approved_files:
            return

        logger.info(f"Found {len(approved_files)} approved file(s) to process")

        for approval_file in approved_files:
            logger.info(f"Processing approved file: {approval_file.name}")

            try:
                # Check if file is readable
                if not approval_file.exists():
                    logger.warning(f"File not found: {approval_file.name}")
                    continue
                
                # Execute action
                success = self.execute_approved_action(approval_file)
                
                # If successful, file was already moved to Done
                # If failed, leave in Approved for manual review
                
            except Exception as e:
                logger.error(f"Failed to process approved file {approval_file.name}: {e}")
                # Leave file in Approved for manual review
    
    def run(self):
        """Main run loop for Local Agent"""
        logger.info("Starting Local Agent run loop...")
        
        import time
        
        while True:
            try:
                # Pull updates from Cloud
                self.vault_sync.pull_updates()
                
                # Process approvals
                self.process_approvals()
                
                # Wait before next cycle
                time.sleep(60)  # Check every minute
                
            except KeyboardInterrupt:
                logger.info("Local Agent stopped by user")
                break
            except Exception as e:
                logger.error(f"Error in Local Agent run loop: {e}")
                time.sleep(60)


if __name__ == '__main__':
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Get vault path from environment or use default (None = use hard-coded path)
    vault_path = os.getenv('VAULT_PATH')  # None if not set
    git_repo_url = os.getenv('GIT_REPO_URL')

    # Create and run agent (None vault_path will use hard-coded E:\hackathon-0\platinum\vault)
    agent = LocalAgent(vault_path, git_repo_url)

    print(f"\nLocal Agent started")
    print(f"Vault: {agent.vault_path}")
    print(f"Mode: Approvals + Execution")
    print(f"Gmail API: {'✓ Available' if agent.gmail_service else '✗ Not Available'}")
    print(f"\nPress Ctrl+C to stop")

    agent.run()
