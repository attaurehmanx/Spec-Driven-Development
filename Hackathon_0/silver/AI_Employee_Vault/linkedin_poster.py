"""
LinkedIn Auto-Poster for AI Employee - Silver Tier
Automatically posts to LinkedIn using browser automation
"""

import time
from pathlib import Path
from datetime import datetime
import json

try:
    from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeout
    PLAYWRIGHT_AVAILABLE = True
except ImportError:
    PLAYWRIGHT_AVAILABLE = False
    print("Error: Playwright not installed. Run: pip install playwright && playwright install chromium")

class LinkedInPoster:
    """Automatically posts to LinkedIn using browser automation"""

    def __init__(self, vault_path: str, session_path: str = './linkedin_session'):
        """
        Initialize LinkedIn poster

        Args:
            vault_path: Path to Obsidian vault
            session_path: Path to store LinkedIn session
        """
        self.vault_path = Path(vault_path)
        self.session_path = Path(session_path)
        self.session_path.mkdir(parents=True, exist_ok=True)

        # Use organized subfolder structure
        self.approved = self.vault_path / 'Approved' / 'linkedin'
        self.done = self.vault_path / 'Done' / 'linkedin'
        self.logs = self.vault_path / 'Logs' / 'linkedin'

        # Ensure directories exist
        self.approved.mkdir(parents=True, exist_ok=True)
        self.done.mkdir(parents=True, exist_ok=True)
        self.logs.mkdir(parents=True, exist_ok=True)

        # Rate limiting
        self.last_post_time = None
        self.min_interval_hours = 12  # Minimum 12 hours between posts

    def check_rate_limit(self) -> bool:
        """Check if enough time has passed since last post"""
        if self.last_post_time is None:
            return True

        hours_since_last = (datetime.now() - self.last_post_time).total_seconds() / 3600
        return hours_since_last >= self.min_interval_hours

    def find_approved_posts(self) -> list:
        """Find approved LinkedIn posts ready to be posted"""
        approved_posts = []

        # Look for any markdown files in the approved/linkedin folder
        for file in self.approved.glob('*.md'):
            try:
                content = file.read_text(encoding='utf-8')

                # Check if it's a LinkedIn post (flexible matching)
                if 'linkedin' in content.lower() or 'post content' in content.lower():
                    approved_posts.append(file)
            except Exception as e:
                print(f"Error reading {file.name}: {e}")

        return approved_posts

    def extract_post_content(self, file_path: Path) -> str:
        """Extract post content from markdown file"""
        content = file_path.read_text(encoding='utf-8')

        # Find the post content section
        lines = content.split('\n')
        post_lines = []
        in_post_section = False

        for line in lines:
            if '### Post Content' in line or '## Post Content' in line:
                in_post_section = True
                continue

            if in_post_section:
                # Stop at next section
                if line.startswith('##') or line.startswith('---'):
                    break

                # Skip empty lines at start
                if not post_lines and not line.strip():
                    continue

                post_lines.append(line)

        return '\n'.join(post_lines).strip()

    def post_to_linkedin(self, post_content: str) -> bool:
        """
        Post content to LinkedIn using browser automation

        Args:
            post_content: The text content to post

        Returns:
            True if successful, False otherwise
        """
        if not PLAYWRIGHT_AVAILABLE:
            print("Error: Playwright not available")
            return False

        try:
            with sync_playwright() as p:
                # Try browsers in order: Edge (guaranteed on Windows) → Chrome → Chromium
                print("Launching browser...")
                try:
                    # Try Edge first (pre-installed on all Windows 10/11)
                    browser = p.chromium.launch_persistent_context(
                        str(self.session_path),
                        headless=False,
                        channel="msedge",  # Use system Edge
                        args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
                    )
                    print("✓ Using Microsoft Edge")
                except:
                    try:
                        # Try Chrome (if user installed it)
                        browser = p.chromium.launch_persistent_context(
                            str(self.session_path),
                            headless=False,
                            channel="chrome",  # Use system Chrome
                            args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
                        )
                        print("✓ Using Google Chrome")
                    except:
                        # Fallback to bundled Chromium
                        browser = p.chromium.launch_persistent_context(
                            str(self.session_path),
                            headless=False,
                            args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
                        )
                        print("✓ Using Chromium")

                page = browser.pages[0] if browser.pages else browser.new_page()

                # Go to LinkedIn
                print("Opening LinkedIn...")
                page.goto('https://www.linkedin.com', timeout=60000)

                # Wait for page to fully load
                print("Waiting for page to load...")
                # Don't wait for networkidle - LinkedIn continuously loads content
                try:
                    page.wait_for_load_state('domcontentloaded', timeout=10000)
                except:
                    pass
                time.sleep(5)

                # Better login detection - check for multiple logged-in indicators
                print("Checking login status...")
                logged_in = False

                # Wait a bit more for page to fully render
                time.sleep(3)

                login_indicators = [
                    'nav.global-nav',  # Main navigation bar
                    '[data-control-name="nav.settings"]',  # Settings menu
                    'img.global-nav__me-photo',  # Profile photo
                    '.feed-identity-module',  # Feed identity section
                    '[data-test-id="nav-settings"]',  # Settings (newer)
                    'button[aria-label*="Me"]',  # Me button
                    'button[id*="ember"]',  # Ember components (LinkedIn uses Ember)
                    '.global-nav__me',  # Me section
                    'a[href*="/mynetwork/"]',  # My Network link
                    'a[href*="/messaging/"]',  # Messaging link
                ]

                for indicator in login_indicators:
                    try:
                        print(f"  Checking: {indicator}")
                        if page.locator(indicator).count() > 0:
                            if page.locator(indicator).first.is_visible(timeout=5000):
                                logged_in = True
                                print(f"✓ Already logged in (detected via: {indicator})")
                                break
                    except Exception as e:
                        print(f"  Not found: {indicator}")
                        continue

                # If still not detected, check URL as fallback
                if not logged_in and '/feed' in page.url:
                    print("✓ Already logged in (detected via URL)")
                    logged_in = True

                if not logged_in:
                    print("\n⚠️  Not logged in to LinkedIn!")
                    print("Please log in manually in the browser window.")
                    print("After logging in, press Enter here to continue...")
                    input()

                    # Wait for login to complete and any redirects to finish
                    print("Waiting for login to complete...")
                    time.sleep(8)
                    # Wait for any post-login navigation to settle
                    try:
                        page.wait_for_load_state('domcontentloaded', timeout=15000)
                    except:
                        pass
                    time.sleep(3)

                # Check if we're already on the feed
                current_url = page.url
                print(f"Current URL: {current_url}")

                if '/feed' not in current_url:
                    # Navigate to feed/home
                    print("Navigating to feed...")
                    try:
                        page.goto('https://www.linkedin.com/feed/', timeout=60000, wait_until='domcontentloaded')
                    except Exception as e:
                        print(f"Navigation warning: {e}")
                        # Try to continue anyway
                        pass
                    time.sleep(5)
                else:
                    print("Already on feed page")
                    time.sleep(3)

                # Find and click "Start a post" button
                print("Looking for post button...")
                time.sleep(3)  # Wait for feed to render

                # Extended list of selectors (LinkedIn changes these frequently)
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
                                        print("✓ Clicked post button")
                                        break
                                except:
                                    continue
                            if clicked:
                                break
                    except Exception as e:
                        continue

                if not clicked:
                    print("\n✗ Could not find post button automatically")
                    print("Please click 'Start a post' manually in the browser window.")
                    print("Press Enter here after clicking...")
                    input()
                    clicked = True

                # Wait for post editor to appear
                print("Waiting for post editor to load...")
                time.sleep(5)

                # Find the text editor
                print("Looking for text editor...")

                editor_selectors = [
                    '.ql-editor',
                    '[contenteditable="true"]',
                    '[role="textbox"]',
                    '.share-creation-state__text-editor',
                    'div[data-placeholder]',
                    '.mentions-texteditor__content',
                ]

                editor_found = False
                for i, selector in enumerate(editor_selectors):
                    try:
                        print(f"  Trying editor selector {i+1}/{len(editor_selectors)}: {selector}")
                        editor = page.locator(selector).first
                        page.wait_for_selector(selector, timeout=5000)

                        if editor.is_visible(timeout=5000):
                            print("✓ Found text editor")

                            # Click to focus
                            editor.click()
                            time.sleep(2)

                            # Type the content
                            print("Typing post content...")
                            editor.fill(post_content)
                            time.sleep(3)

                            editor_found = True
                            break
                    except Exception as e:
                        continue

                if not editor_found:
                    print("\n✗ Could not find text editor automatically")
                    print("Please paste your content manually in the browser window.")
                    print("\nYour post content:")
                    print("-" * 60)
                    print(post_content)
                    print("-" * 60)
                    print("\nPress Enter here after pasting...")
                    input()
                    editor_found = True

                # Find and click Post button
                print("Looking for Post button...")
                time.sleep(2)

                post_submit_selectors = [
                    'button:has-text("Post")',
                    'button:has-text("post")',
                    'button[aria-label*="Post"]',
                    'button[aria-label*="post"]',
                    '.share-actions__primary-action',
                    '[data-control-name="share.post"]',
                    'button.share-actions__primary-action',
                    'button[type="submit"]',
                ]

                posted = False
                for i, selector in enumerate(post_submit_selectors):
                    try:
                        print(f"  Trying post button selector {i+1}/{len(post_submit_selectors)}: {selector}")

                        # Wait for button to be enabled
                        time.sleep(2)

                        button = page.locator(selector).first
                        if button.is_visible(timeout=5000):
                            # Check if button is enabled
                            is_disabled = button.get_attribute('disabled')
                            if is_disabled:
                                print(f"  Button is disabled, waiting...")
                                time.sleep(3)

                            print("✓ Found Post button")

                            # Click to post
                            button.click()
                            print("✓ Clicked Post button")

                            # Wait for post to complete
                            print("Waiting for post to complete...")
                            time.sleep(8)

                            posted = True
                            break
                    except Exception as e:
                        continue

                if not posted:
                    print("\n✗ Could not click Post button automatically")
                    print("Please click 'Post' manually in the browser window.")
                    print("Press Enter here after posting...")
                    input()
                    posted = True  # Assume user posted manually

                print("\n✓ Post completed successfully!")
                print("Keeping browser open for 5 seconds...")
                time.sleep(5)

                browser.close()
                return posted

        except Exception as e:
            print(f"Error posting to LinkedIn: {e}")
            return False

    def log_post(self, file_path: Path, success: bool, error: str = None):
        """Log posting activity"""
        log_file = self.logs / f"{datetime.now().strftime('%Y-%m-%d')}.json"

        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'action_type': 'linkedin_post',
            'file': file_path.name,
            'success': success,
            'error': error
        }

        # Read existing logs
        logs = []
        if log_file.exists():
            try:
                logs = json.loads(log_file.read_text())
            except:
                logs = []

        # Append new log
        logs.append(log_entry)

        # Write back
        log_file.write_text(json.dumps(logs, indent=2))

    def process_approved_posts(self):
        """Process all approved LinkedIn posts"""
        print("\n" + "="*60)
        print("LinkedIn Auto-Poster")
        print("="*60)

        # Check rate limit
        if not self.check_rate_limit():
            hours_since = (datetime.now() - self.last_post_time).total_seconds() / 3600
            hours_remaining = self.min_interval_hours - hours_since
            print(f"\n⚠️  Rate limit: Must wait {hours_remaining:.1f} more hours")
            print(f"Last post: {self.last_post_time.strftime('%Y-%m-%d %H:%M')}")
            return

        # Find approved posts
        approved_posts = self.find_approved_posts()

        if not approved_posts:
            print("\nNo approved LinkedIn posts found.")
            print(f"Place approved posts in: {self.approved}")
            return

        print(f"\nFound {len(approved_posts)} approved post(s)")

        # Process each post
        for post_file in approved_posts:
            print(f"\n{'='*60}")
            print(f"Processing: {post_file.name}")
            print(f"{'='*60}")

            try:
                # Extract content
                content = self.extract_post_content(post_file)

                if not content:
                    print("✗ Could not extract post content")
                    continue

                print(f"\nPost preview (first 100 chars):")
                print(f"{content[:100]}...")

                # Confirm before posting
                print("\n⚠️  Ready to post to LinkedIn")
                print("This will open a browser window.")
                response = input("Continue? (yes/no): ").lower()

                if response != 'yes':
                    print("Skipped.")
                    continue

                # Post to LinkedIn
                print("\nPosting to LinkedIn...")
                success = self.post_to_linkedin(content)

                if success:
                    print("\n✓ Successfully posted to LinkedIn!")

                    # Move to Done
                    done_path = self.done / post_file.name
                    post_file.rename(done_path)
                    print(f"✓ Moved to Done: {done_path.name}")

                    # Log success
                    self.log_post(post_file, True)

                    # Update last post time
                    self.last_post_time = datetime.now()

                    # Only post one at a time
                    print(f"\n✓ Post complete. Rate limit: {self.min_interval_hours} hours")
                    break
                else:
                    print("\n✗ Failed to post")
                    self.log_post(post_file, False, "Posting failed")

            except Exception as e:
                print(f"\n✗ Error: {e}")
                self.log_post(post_file, False, str(e))

    def run_once(self):
        """Run poster once (for manual execution)"""
        self.process_approved_posts()

    def run_scheduled(self):
        """Run poster on schedule (for cron/scheduler)"""
        print(f"LinkedIn Auto-Poster - Scheduled Run")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self.process_approved_posts()


if __name__ == '__main__':
    import sys

    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'
    session_path = sys.argv[2] if len(sys.argv) > 2 else './linkedin_session'

    poster = LinkedInPoster(vault_path, session_path)
    poster.run_once()
