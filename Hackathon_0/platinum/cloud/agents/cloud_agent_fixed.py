"""
cloud/agents/cloud_agent.py
Cloud Agent: Drafts Only (Email triage, social drafts, Odoo drafts)

FIXED VERSION with proper imports and real API integrations
"""

import os
import sys
import json
import logging
import base64
import re
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, List, Optional
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from shared.claim_manager.claim_manager import ClaimManager
from shared.vault_sync.git_sync import VaultSync

# Try to import Google API libs
try:
    from google.oauth2.credentials import Credentials
    from google_auth_oauthlib.flow import InstalledAppFlow
    from google.auth.transport.requests import Request
    from googleapiclient.discovery import build
    GMAIL_AVAILABLE = True
except ImportError:
    GMAIL_AVAILABLE = False
    print("Warning: Google API libs not installed. Install: pip install google-auth google-auth-oauthlib google-api-python-client")

# Try to import social media libs
try:
    import requests
    SOCIAL_AVAILABLE = True
except ImportError:
    SOCIAL_AVAILABLE = False
    print("Warning: requests library not installed. Install: pip install requests")

# Try to import Gemini AI for content generation
try:
    import google.generativeai as genai
    from social_platform.agents.gemini_content_generator import GeminiContentGenerator, AIContentWorkflow
    GEMINI_AVAILABLE = True
    print("✓ Gemini AI available for content generation")
except ImportError:
    GEMINI_AVAILABLE = False
    print("Note: Gemini AI not installed. Install: pip install google-generativeai")
    print("Get FREE API key: https://aistudio.google.com/app/apikey")

logger = logging.getLogger(__name__)


class CloudAgent:
    """
    Cloud Agent: Drafts Only
    
    Responsibilities:
    - Email triage and draft replies
    - Social media post drafts
    - Odoo draft invoices and payments
    - NEVER executes send/post/payment actions
    - NEVER stores WhatsApp sessions
    - NEVER stores banking credentials
    - ALWAYS creates approval requests for Local
    """
    
    ALLOWED_ACTIONS = [
        'read_email',
        'draft_email_reply',
        'read_social_mentions',
        'draft_social_post',
        'read_odoo_data',
        'draft_invoice',
        'draft_payment',
        'create_approval_request',
    ]
    
    FORBIDDEN_ACTIONS = [
        'send_email',
        'post_social',
        'post_invoice',
        'execute_payment',
        'whatsapp_send',
        'banking_login',
    ]
    
    def __init__(self, vault_path: str = None, git_repo_url: Optional[str] = None):
        """
        Initialize Cloud Agent
        
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
        
        self.agent_name = 'cloud_agent'
        
        logger.info(f"Vault path: {self.vault_path}")
        
        # Initialize claim manager
        self.claim_manager = ClaimManager(str(self.vault_path), self.agent_name)
        
        # Initialize vault sync
        self.vault_sync = VaultSync(str(self.vault_path), git_repo_url)
        self.vault_sync.initialize()
        
        # Ensure folders exist
        (self.vault_path / 'Pending_Approval').mkdir(parents=True, exist_ok=True)
        (self.vault_path / 'Updates').mkdir(parents=True, exist_ok=True)
        
        # Gmail service
        self.gmail_service = None
        self._init_gmail()

        # Gemini AI content generator
        self.ai_content_workflow = None
        if GEMINI_AVAILABLE:
            try:
                self.ai_content_workflow = AIContentWorkflow(str(self.vault_path))
                logger.info("✓ Gemini AI content generation enabled")
            except ValueError as e:
                logger.warning(f"Gemini API key not configured: {e}")
                logger.info("Add GEMINI_API_KEY to .env to enable AI content generation")

        logger.info(f"Cloud Agent initialized (vault: {self.vault_path})")
    
    def _init_gmail(self):
        """Initialize Gmail API connection"""
        if not GMAIL_AVAILABLE:
            logger.warning("Gmail API not available")
            return
        
        try:
            import pickle
            from googleapiclient.discovery import build
            
            # Get script location to find credentials in project root
            script_dir = Path(__file__).resolve().parent  # E:\hackathon-0\platinum\cloud\agents
            project_root = script_dir.parent.parent  # E:\hackathon-0\platinum
            
            # Load credentials from environment or file (project root)
            # Use SAME token file as Local Agent (gmail_token.json)
            credentials_path = os.getenv('GMAIL_CREDENTIALS_PATH', str(project_root / 'credentials.json'))
            token_path = os.getenv('GMAIL_TOKEN_PATH', str(project_root / 'gmail_token.json'))
            
            # Use correct Gmail scopes (readonly for Cloud, send for Local)
            # Cloud Agent needs readonly to read emails
            SCOPES = ['https://www.googleapis.com/auth/gmail.readonly', 'https://www.googleapis.com/auth/gmail.send']
            
            creds = None
            if Path(token_path).exists():
                with open(token_path, 'rb') as token:
                    creds = pickle.load(token)
            
            if not creds or not creds.valid:
                if creds and creds.expired and creds.refresh_token:
                    creds.refresh(Request())
                else:
                    if Path(credentials_path).exists():
                        logger.info(f"Found credentials: {credentials_path}")
                        flow = InstalledAppFlow.from_client_secrets_file(
                            credentials_path, SCOPES
                        )
                        creds = flow.run_local_server(port=0)
                    else:
                        logger.warning(f"Gmail credentials not found: {credentials_path}")
                        logger.info(f"Please copy credentials.json from Gold tier to: {project_root}")
                        return
            
            self.gmail_service = build('gmail', 'v1', credentials=creds)
            logger.info("Gmail API initialized")
            
        except Exception as e:
            logger.error(f"Gmail initialization failed: {e}")
            import traceback
            traceback.print_exc()
            self.gmail_service = None
    
    def process_email(self, email_data: Dict[str, Any]) -> Optional[Path]:
        """
        Triage email and draft reply
        
        Args:
            email_data: Email data with from, subject, body, etc.
            
        Returns:
            Path to approval file if reply drafted, None otherwise
        """
        logger.info(f"Processing email from {email_data.get('from', 'unknown')}")
        
        # Analyze email
        analysis = self._analyze_email(email_data)
        
        # Draft reply if needed
        if analysis.get('needs_reply'):
            draft = self._draft_reply(email_data, analysis)
            
            # Create approval request
            approval_file = self.create_approval_request({
                'type': 'email_send',
                'draft': draft,
                'recipient': email_data['from'],
                'subject': f"Re: {email_data['subject']}",
                'original_email': email_data,
            })
            
            return approval_file
        
        return None
    
    def _analyze_email(self, email_data: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze email to determine if reply needed"""
        body = email_data.get('body', '').lower()
        subject = email_data.get('subject', '').lower()
        
        logger.info(f"Analyzing email - Subject: {subject[:50]}")
        logger.info(f"Body length: {len(body)} characters")
        
        # Keywords indicating reply needed
        reply_keywords = ['question', 'help', 'request', 'invoice', 'payment',
                         'when', 'how', 'what', 'can you', 'could you', 'progress', 'status']
        
        needs_reply = any(keyword in body or keyword in subject
                         for keyword in reply_keywords)
        
        logger.info(f"Needs reply: {needs_reply}")
        
        # Priority calculation
        priority = 'medium'
        if any(word in body for word in ['urgent', 'asap', 'emergency']):
            priority = 'high'
        elif any(word in body for word in ['newsletter', 'update']):
            priority = 'low'
        
        return {
            'needs_reply': needs_reply,
            'priority': priority,
            'category': self._categorize_email(body, subject),
        }
    
    def _categorize_email(self, body: str, subject: str) -> str:
        """Categorize email"""
        if any(word in body for word in ['invoice', 'payment', 'bill']):
            return 'financial'
        elif any(word in body for word in ['meeting', 'call', 'schedule']):
            return 'scheduling'
        elif any(word in body for word in ['question', 'help', 'support']):
            return 'support'
        else:
            return 'general'
    
    def _draft_reply(self, email_data: Dict[str, Any], analysis: Dict[str, Any]) -> str:
        """Draft email reply"""
        category = analysis.get('category', 'general')
        
        # Template-based drafts
        templates = {
            'financial': """Dear {name},

Thank you for your inquiry regarding the financial matter.

I have received your message and will process it shortly.

Best regards,
AI Employee""",
            
            'scheduling': """Dear {name},

Thank you for reaching out about scheduling.

I will check the availability and get back to you soon.

Best regards,
AI Employee""",
            
            'support': """Dear {name},

Thank you for contacting support.

I understand your concern and will assist you promptly.

Best regards,
AI Employee""",
            
            'general': """Dear {name},

Thank you for your email.

I have received your message and will respond shortly.

Best regards,
AI Employee""",
        }
        
        template = templates.get(category, templates['general'])
        sender_name = email_data.get('from', 'Sender').split('@')[0]
        
        return template.format(name=sender_name)
    
    def process_social_mention(self, mention_data: Dict[str, Any]) -> Optional[Path]:
        """Process social media mention and draft response"""
        logger.info(f"Processing {mention_data.get('platform', 'unknown')} mention")
        
        # Draft response
        response_draft = f"""Thank you for your message! We appreciate your engagement.

Our team will review your inquiry and respond appropriately.

Best regards,
AI Employee"""
        
        # Create approval request
        approval_file = self.create_approval_request({
            'type': 'social_reply',
            'platform': mention_data['platform'],
            'draft': response_draft,
            'original_mention': mention_data,
        })
        
        return approval_file
    
    def draft_social_post(self, topic: str, content: str = None, 
                         platforms: List[str] = None, style: str = "professional",
                         use_ai: bool = False) -> Optional[Path]:
        """
        Draft social media post
        
        Args:
            topic: What to post about
            content: Optional pre-written content (if None, AI will generate)
            platforms: List of platforms to post to
            style: Content style (professional, casual, funny, etc.)
            use_ai: If True, use Gemini to generate content
            
        Returns:
            Path to approval file
        """
        logger.info(f"Drafting social post about: {topic}")
        
        # Use AI to generate content if requested or if no content provided
        if use_ai or (not content and self.ai_content_workflow):
            if self.ai_content_workflow and GEMINI_AVAILABLE:
                logger.info("Using Gemini AI to generate social media content...")
                
                # Generate AI content
                platforms = platforms or ['facebook', 'instagram', 'twitter']
                approval_files = self.ai_content_workflow.create_ai_generated_post(
                    topic=topic,
                    platforms=platforms,
                    style=style
                )
                
                if approval_files:
                    logger.info(f"Created {len(approval_files)} AI-generated approval files")
                    return approval_files[0] if approval_files else None
            
            # Fallback if AI not available
            if not content:
                content = f"Exciting news about {topic}! Stay tuned for more details. #AI #Innovation"
        
        # Create approval request with content
        approval_file = self.create_approval_request({
            'type': 'social_post',
            'topic': topic,
            'content': content,
            'platforms': platforms or ['facebook', 'instagram', 'twitter'],
            'style': style,
            'ai_generated': use_ai,
        })

        return approval_file
    
    def generate_ai_content(self, topic: str, platforms: List[str] = None, 
                           style: str = "professional") -> Optional[Path]:
        """
        Generate social media content using Gemini AI
        
        Args:
            topic: What to post about
            platforms: List of platforms to post to
            style: Content style
            
        Returns:
            Path to approval file with AI-generated options
        """
        if not self.ai_content_workflow or not GEMINI_AVAILABLE:
            logger.warning("AI content generation not available")
            return self.draft_social_post(topic, platforms=platforms, style=style)
        
        logger.info(f"Generating AI content for: {topic}")
        
        platforms = platforms or ['facebook', 'instagram', 'twitter', 'linkedin']
        
        # Generate AI content for all platforms
        approval_files = self.ai_content_workflow.create_ai_generated_post(
            topic=topic,
            platforms=platforms,
            style=style
        )
        
        logger.info(f"Generated AI content for {len(approval_files)} platforms")
        
        return approval_files[0] if approval_files else None
    
    def draft_invoice(self, invoice_data: Dict[str, Any]) -> Optional[Path]:
        """Draft invoice in Odoo"""
        logger.info(f"Drafting invoice for {invoice_data.get('customer', 'unknown')}")
        
        # Create approval request
        approval_file = self.create_approval_request({
            'type': 'invoice_draft',
            'customer': invoice_data['customer'],
            'amount': invoice_data['amount'],
            'items': invoice_data.get('items', []),
            'due_date': invoice_data.get('due_date'),
        })
        
        return approval_file
    
    def draft_payment(self, payment_data: Dict[str, Any]) -> Optional[Path]:
        """Draft payment"""
        logger.info(f"Drafting payment to {payment_data.get('recipient', 'unknown')}")
        
        # Create approval request
        approval_file = self.create_approval_request({
            'type': 'payment_draft',
            'recipient': payment_data['recipient'],
            'amount': payment_data['amount'],
            'reference': payment_data.get('reference', ''),
        })
        
        return approval_file
    
    def create_approval_request(self, request_data: Dict[str, Any]) -> Path:
        """Create approval request file in Pending_Approval folder"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        request_type = request_data.get('type', 'unknown')
        
        filename = f"{request_type.upper()}_{timestamp}.md"
        filepath = self.vault_path / 'Pending_Approval' / filename
        
        # Also write to Updates folder for sync
        update_path = self.vault_path / 'Updates' / filename
        
        content = f"""---
type: {request_type}
created: {datetime.now().isoformat()}
created_by: {self.agent_name}
status: pending
requires_local_execution: true
---

# Approval Required

## Action
{request_type.replace('_', ' ').title()}

## Details
{json.dumps(request_data, indent=2, default=str)}

## To Approve
Move this file to /Approved folder

## Note
This action requires Local execution (send/post/payment)
Cloud agent cannot execute this action.
"""
        
        # Write to Pending_Approval
        filepath.write_text(content)
        logger.info(f"Created approval request: {filepath.name}")
        
        # Write to Updates for sync
        update_path.write_text(content)
        logger.info(f"Created update file: {update_path.name}")
        
        # Sync to vault
        self.vault_sync.push_updates(f"New approval request: {filename}")
        
        return filepath
    
    def process_needs_action_folder(self):
        """Process all files in Needs_Action folder"""
        logger.info("Processing Needs_Action folder...")
        
        # Get available tasks
        available_tasks = self.claim_manager.get_available_tasks()
        logger.info(f"Found {len(available_tasks)} available tasks")
        
        for task_file in available_tasks:
            # Try to claim task
            if self.claim_manager.claim_task(task_file):
                logger.info(f"Claimed task: {task_file.name}")
                
                try:
                    # Process based on task type
                    metadata = self.claim_manager.get_task_metadata(task_file)
                    task_type = metadata.get('type', 'unknown')
                    
                    if task_type == 'email':
                        self._process_email_task(task_file, metadata)
                    elif task_type == 'social':
                        self._process_social_task(task_file, metadata)
                    elif task_type == 'odoo':
                        self._process_odoo_task(task_file, metadata)
                    else:
                        logger.warning(f"Unknown task type: {task_type}")
                    
                    # Release task as done
                    claimed_file = self.claim_manager.in_progress_folder / task_file.name
                    if claimed_file.exists():
                        self.claim_manager.release_task(claimed_file, status='done')
                
                except Exception as e:
                    logger.error(f"Failed to process task {task_file.name}: {e}")
                    # Release task as failed
                    claimed_file = self.claim_manager.in_progress_folder / task_file.name
                    if claimed_file.exists():
                        self.claim_manager.release_task(claimed_file, status='failed')
            else:
                logger.debug(f"Task already claimed: {task_file.name}")
    
    def _process_email_task(self, task_file: Path, metadata: Dict[str, Any]):
        """Process email task"""
        content = task_file.read_text()
        # Extract email data from file
        email_data = {
            'from': metadata.get('from', 'unknown'),
            'subject': metadata.get('subject', 'No Subject'),
            'body': content,
        }
        self.process_email(email_data)
    
    def _process_social_task(self, task_file: Path, metadata: Dict[str, Any]):
        """
        Process social media task
        
        Supports both manual content and AI-generated content
        """
        content = task_file.read_text()
        
        # Check if AI generation is requested
        use_ai = metadata.get('use_ai', 'false').lower() == 'true'
        auto_generate = metadata.get('auto_generate', 'false').lower() == 'true'
        
        # Get parameters
        topic = metadata.get('topic', metadata.get('subject', 'Social Media Post'))
        platforms_str = metadata.get('platforms', 'facebook,instagram,twitter')
        style = metadata.get('style', 'professional')
        
        # Parse platforms
        platforms = [p.strip() for p in platforms_str.split(',')]
        
        # Check if content is provided or should be AI-generated
        provided_content = metadata.get('content')
        if not provided_content:
            # Extract content from file body (after frontmatter)
            if '---' in content:
                parts = content.split('---', 2)
                if len(parts) > 2:
                    provided_content = parts[2].strip()
        
        logger.info(f"Processing social task - Topic: {topic}, AI: {use_ai}")
        
        # Use AI generation if requested or if no content provided
        if use_ai or auto_generate or (not provided_content and self.ai_content_workflow):
            if self.ai_content_workflow and GEMINI_AVAILABLE:
                logger.info("Using Gemini AI to generate social media content...")
                
                # Generate AI content for all platforms
                approval_files = self.ai_content_workflow.create_ai_generated_post(
                    topic=topic,
                    platforms=platforms,
                    style=style
                )
                
                if approval_files:
                    logger.info(f"Created {len(approval_files)} AI-generated approval files")
                    
                    # Release task as done
                    claimed_file = self.claim_manager.in_progress_folder / task_file.name
                    if claimed_file.exists():
                        self.claim_manager.release_task(claimed_file, status='done')
                    return
        
        # Fallback: Create simple approval request
        mention_data = {
            'platform': platforms[0] if platforms else 'facebook',
            'text': provided_content or content,
            'user': metadata.get('user', 'unknown'),
            'topic': topic,
            'style': style,
        }
        self.process_social_mention(mention_data)
        
        # Release task as done
        claimed_file = self.claim_manager.in_progress_folder / task_file.name
        if claimed_file.exists():
            self.claim_manager.release_task(claimed_file, status='done')
    
    def _process_odoo_task(self, task_file: Path, metadata: Dict[str, Any]):
        """Process Odoo task"""
        content = task_file.read_text()
        # Extract Odoo data from file
        odoo_data = {
            'type': metadata.get('odoo_type', 'unknown'),
            'customer': metadata.get('customer', 'unknown'),
            'amount': metadata.get('amount', 0),
        }
        if odoo_data['type'] == 'invoice':
            self.draft_invoice(odoo_data)
        elif odoo_data['type'] == 'payment':
            self.draft_payment(odoo_data)
    
    def run(self):
        """Main run loop for Cloud Agent"""
        logger.info("Starting Cloud Agent run loop...")
        
        import time
        
        # Track processed email IDs to avoid duplicates
        self.processed_ids = set()
        
        while True:
            try:
                # 1. Check Gmail for NEW unread emails
                self._check_gmail_for_new_emails()
                
                # 2. Process Needs_Action folder
                self.process_needs_action_folder()
                
                # 3. Sync vault
                self.vault_sync.push_updates("Cloud agent processing")
                
                # 4. Wait before next cycle
                time.sleep(60)  # Check every minute
                
            except KeyboardInterrupt:
                logger.info("Cloud Agent stopped by user")
                break
            except Exception as e:
                logger.error(f"Error in Cloud Agent run loop: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(60)
    
    def _check_gmail_for_new_emails(self):
        """
        Check Gmail API for new unread important emails
        
        This is the ACTUAL Gmail watching functionality (from Gold tier)
        """
        if not self.gmail_service:
            logger.debug("Gmail service not available - skipping Gmail check")
            return
        
        try:
            logger.info("Checking Gmail for new unread emails...")
            
            # Query for unread important emails
            results = self.gmail_service.users().messages().list(
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
                logger.info(f"Found {len(new_messages)} new unread email(s)")
                
                # Create action files for each new email
                for message in new_messages:
                    self._create_email_action_file(message)
                    self.processed_ids.add(message['id'])
            else:
                logger.debug("No new unread emails")
            
        except Exception as e:
            logger.error(f"Error checking Gmail: {e}")
    
    def _create_email_action_file(self, message: dict):
        """
        Create action file for Gmail message
        
        Args:
            message: Gmail message dict with 'id' key
        """
        try:
            # Get full message details
            msg = self.gmail_service.users().messages().get(
                userId='me',
                id=message['id'],
                format='full'
            ).execute()
            
            # Extract headers
            headers = {
                h['name']: h['value']
                for h in msg['payload']['headers']
            }
            
            from_email = headers.get('From', 'Unknown')
            subject = headers.get('Subject', 'No Subject')
            date = headers.get('Date', datetime.now().isoformat())
            
            # Extract body snippet
            snippet = msg.get('snippet', '')
            
            # Calculate priority
            priority = self._calculate_email_priority(subject, snippet, from_email)
            
            # Create email folder
            email_folder = self.vault_path / 'Needs_Action' / 'email'
            email_folder.mkdir(parents=True, exist_ok=True)
            
            # Create content
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            safe_subject = "".join(c for c in subject if c.isalnum() or c in (' ', '-', '_'))[:50]
            filename = f'EMAIL_{timestamp}_{safe_subject}.md'
            filepath = email_folder / filename
            
            content = f"""---
type: email
from: {from_email}
subject: {subject}
date: {date}
priority: {priority}
message_id: {message['id']}
---

## Email from {from_email}

**Subject**: {subject}

**Date**: {date}

**Priority**: {priority}

### Preview
{snippet}

### Suggested Actions
- [ ] Read full email
- [ ] Draft reply
- [ ] Forward if needed
- [ ] Archive after processing
"""
            
            filepath.write_text(content, encoding='utf-8')
            logger.info(f"Created email action file: {filename}")
            
        except Exception as e:
            logger.error(f"Error creating action file for email: {e}")
    
    def _calculate_email_priority(self, subject: str, snippet: str, from_email: str) -> str:
        """Calculate email priority based on content"""
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
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Get vault path from environment or use default (None = use hard-coded path)
    vault_path = os.getenv('VAULT_PATH')  # None if not set
    git_repo_url = os.getenv('GIT_REPO_URL')

    # Create and run agent (None vault_path will use hard-coded E:\hackathon-0\platinum\vault)
    agent = CloudAgent(vault_path, git_repo_url)

    print(f"\nCloud Agent started")
    print(f"Vault: {agent.vault_path}")
    print(f"Mode: Drafts Only (no sending)")
    print(f"\nPress Ctrl+C to stop")

    agent.run()
