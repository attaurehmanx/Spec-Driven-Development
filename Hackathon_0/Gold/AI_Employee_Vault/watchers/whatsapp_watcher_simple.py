"""
WhatsApp Watcher - Simple Version
Uses page-wide text search instead of specific selectors
"""
from pathlib import Path
from datetime import datetime
from base_watcher import BaseWatcher
import json
import time

try:
    from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeout
    PLAYWRIGHT_AVAILABLE = True
except ImportError:
    PLAYWRIGHT_AVAILABLE = False
    print("WARNING: Playwright not installed. Run: pip install playwright")

class WhatsAppWatcherSimple(BaseWatcher):
    """WhatsApp watcher with simple page-wide text search"""

    def __init__(self, vault_path: str, session_path: str = None, check_interval: int = 30):
        # Set default session path if not provided
        if session_path is None:
            session_path = 'E:/hackathon-0/Gold/AI_Employee_Vault/whatsapp_session'
        
        super().__init__(vault_path, check_interval)
        self.session_path = Path(session_path)
        self.session_path.mkdir(parents=True, exist_ok=True)
        
        # Create whatsapp subfolder in Needs_Action
        self.whatsapp_folder = self.needs_action / 'whatsapp'
        self.whatsapp_folder.mkdir(parents=True, exist_ok=True)

        # Check if running in quiet mode
        import sys
        self.quiet_mode = '--quiet' in sys.argv

        # Browser session variables (persistent like Gmail)
        self.playwright = None
        self.browser = None
        self.page = None
        self.is_authenticated = False

        if not PLAYWRIGHT_AVAILABLE:
            self.logger.error("Playwright not available")
            return

        # Initialize browser session once
        self._init_browser()

    def _init_browser(self):
        """Initialize persistent browser session (opens once, stays open)"""
        if not PLAYWRIGHT_AVAILABLE:
            return

        try:
            self.playwright = sync_playwright().start()

            # Try to use system browser (Edge first, then Chrome, then Chromium)
            browser_launched = False
            channels_to_try = ['msedge', 'chrome', 'chromium']

            for channel in channels_to_try:
                try:
                    if not self.quiet_mode:
                        print(f"Trying to launch {channel}...")

                    # Launch browser with persistent context (always visible - WhatsApp blocks headless)
                    self.browser = self.playwright.chromium.launch_persistent_context(
                        str(self.session_path),
                        headless=False,  # Must be False - WhatsApp blocks headless browsers
                        channel=channel,  # Use system browser
                        args=[
                            '--no-sandbox',
                            '--disable-blink-features=AutomationControlled',
                            '--window-position=100,100',  # Force window on-screen
                            '--window-size=1200,900',  # Set reasonable window size
                        ]
                    )
                    browser_launched = True
                    if not self.quiet_mode:
                        print(f"✅ Successfully launched {channel}")
                    break
                except Exception as e:
                    if not self.quiet_mode:
                        print(f"❌ Could not launch {channel}: {e}")
                    continue

            if not browser_launched:
                self.logger.error("Could not launch any browser")
                return

            self.page = self.browser.pages[0] if self.browser.pages else self.browser.new_page()
            self.page.goto('https://web.whatsapp.com', timeout=90000)

            # Try multiple selectors to detect WhatsApp loaded
            loaded = False
            selectors_to_try = [
                '[aria-label="Chat list"]',
                '#pane-side',
                '[data-testid="chat-list"]',
                'div[role="application"]'
            ]

            for i in range(18):  # Try for 90 seconds (18 * 5 seconds)
                time.sleep(5)

                for selector in selectors_to_try:
                    try:
                        element = self.page.query_selector(selector)
                        if element:
                            loaded = True
                            break
                    except:
                        continue

                if loaded:
                    break

                elapsed = (i + 1) * 5

            if not loaded:
                self.is_authenticated = False
            else:
                if self.quiet_mode:
                    print(f"[WHATSAPP] Browser authenticated")
                else:
                    print(f"✅ WhatsApp loaded successfully!")
                    print(f"   🎉 Authentication successful!")
                    print(f"   Browser will stay open in background.\n")
                self.is_authenticated = True
                self.logger.info("WhatsApp session initialized successfully")

        except Exception as e:
            self.logger.error(f"Error initializing browser: {e}")
            self.is_authenticated = False

    def check_for_updates(self) -> list:
        """Check for ALL unread WhatsApp messages using persistent session"""
        if not PLAYWRIGHT_AVAILABLE or not self.is_authenticated or not self.page:
            self.logger.debug(f"Skipping check: PLAYWRIGHT={PLAYWRIGHT_AVAILABLE}, authenticated={self.is_authenticated}, page={self.page is not None}")
            return []

        messages = []

        try:
            # Don't reload - just check the current page state
            time.sleep(2)  # Brief wait for page to settle

            # Get all chat list items - WhatsApp Web uses div[tabindex="0"] based on debug
            try:
                # First check if #pane-side exists
                pane_side = self.page.query_selector('#pane-side')
                if not pane_side:
                    self.logger.debug("pane-side not found - WhatsApp may not be fully loaded")
                    return []
                
                # Primary selector based on debug output: div[tabindex="0"]
                chat_items = self.page.query_selector_all('#pane-side div[tabindex="0"]')
                self.logger.debug(f"Found {len(chat_items)} chat items")

                if not chat_items or len(chat_items) == 0:
                    self.logger.debug("No chat items found on WhatsApp Web")
                    return []

            except Exception as e:
                self.logger.debug(f"Error finding chat items: {e}")
                return []

            self.logger.debug(f"Checking {len(chat_items)} chats for unread messages")

            # Check each chat for unread indicator
            for idx, chat in enumerate(chat_items[:30]):  # Check first 30 chats
                try:
                    text = chat.inner_text()
                    if not text or len(text.strip()) < 5:
                        continue

                    # Get aria-label for debugging
                    aria_label = chat.get_attribute('aria-label') or ""

                    # Check for unread indicators
                    is_unread = False

                    # Method 1: Look for unread badge within this chat (aria-label contains "unread")
                    unread_elem = chat.query_selector('[aria-label*="unread"]')
                    if unread_elem:
                        is_unread = True
                        badge_text = unread_elem.inner_text()
                        self.logger.debug(f"Chat {idx}: Found unread badge with text: {badge_text}")

                    # Method 2: Check if chat's own aria-label contains "unread"
                    if not is_unread and 'unread' in aria_label.lower():
                        is_unread = True
                        self.logger.debug(f"Chat {idx}: Found 'unread' in aria-label: {aria_label}")

                    if is_unread:
                        self.logger.debug(f"Chat {idx} is UNREAD. Full text: {text[:200]}")
                        
                        # Parse the chat info from text
                        # Format from debug: "2 unread messages | Boss | 00:41 | Hi2 | 2"
                        # Or: "4 unread messages | WhatsApp | Tuesday | No one can see..."
                        lines = [line.strip() for line in text.split('\n') if line.strip()]
                        self.logger.debug(f"Chat {idx} lines: {lines[:6]}")

                        # Try to extract sender name and message
                        chat_name = "Unknown"
                        time_str = "Unknown"
                        message_text = text[:200]
                        
                        # Look for patterns in the text
                        text_combined = ' '.join(lines)
                        
                        # Check if first line is "X unread messages"
                        if lines and 'unread messages' in lines[0].lower():
                            # Skip the "X unread messages" part
                            # Next part should be the sender name
                            if len(lines) >= 2:
                                chat_name = lines[1]  # Second line is sender name
                                # Third line might be time
                                if len(lines) >= 3:
                                    time_str = lines[2]
                                    # Rest is message
                                    message_lines = lines[3:]
                                    message_text = ' '.join(message_lines) if message_lines else lines[-1]
                        elif len(lines) >= 3:
                            # Standard format: Name, Time, Message
                            chat_name = lines[0]
                            for i, line in enumerate(lines[1:], 1):
                                if (':' in line and len(line) <= 8) or \
                                   line.lower() in ['yesterday', 'today'] or \
                                   (len(line) <= 10 and '/' in line):
                                    time_str = line
                                    message_start_idx = i + 1
                                    break
                            else:
                                message_start_idx = 1
                            message_lines = lines[message_start_idx:]
                            message_text = ' '.join(message_lines) if message_lines else lines[-1]
                        
                        # Skip system messages
                        if 'whatsapp' in chat_name.lower() or \
                           'security code' in message_text.lower() or \
                           'end-to-end encryption' in message_text.lower():
                            self.logger.debug(f"Chat {idx}: Skipping system message from {chat_name}")
                            continue

                        # Create unique ID
                        chat_id = f"whatsapp_{chat_name}_{hash(text)}"

                        # Use base class processed_ids
                        if chat_id not in self.processed_ids:
                            # Create individual message for each chat
                            messages.append({
                                'id': f"whatsapp_{chat_name}_{datetime.now().strftime('%Y%m%d%H%M%S')}",
                                'name': chat_name,
                                'time': time_str,
                                'message': message_text[:300],
                                'chat_id': chat_id,
                                'timestamp': datetime.now().isoformat()
                            })
                            self.processed_ids.add(chat_id)
                            self.logger.info(f"Found unread chat from: {chat_name}")
                        else:
                            self.logger.debug(f"Chat {idx}: Already processed (chat_id={chat_id})")

                except Exception as e:
                    self.logger.debug(f"Error processing chat {idx}: {e}")
                    import traceback
                    traceback.print_exc()
                    continue

        except Exception as e:
            self.logger.error(f"Error checking WhatsApp: {e}")
            # Try to reinitialize browser on error
            self.logger.info("Attempting to reinitialize browser session...")
            self._cleanup_browser()
            self._init_browser()

        return messages

    def _cleanup_browser(self):
        """Clean up browser resources"""
        try:
            if self.browser:
                self.browser.close()
            if self.playwright:
                self.playwright.stop()
        except Exception as e:
            self.logger.error(f"Error cleaning up browser: {e}")
        finally:
            self.browser = None
            self.page = None
            self.playwright = None
            self.is_authenticated = False

    def __del__(self):
        """Cleanup when object is destroyed"""
        self._cleanup_browser()

    def create_action_file(self, message) -> Path:
        """Create action file for a single WhatsApp unread message"""
        timestamp = datetime.fromisoformat(message['timestamp'])

        content = f"""---
type: whatsapp_message
received: {message['timestamp']}
sender: {message['name']}
priority: normal
status: pending
---

## WhatsApp Message

**From**: {message['name']}

**Time**: {message['time']}

**Date**: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}

**Status**: Unread

---

### Message Preview

{message['message']}

---

## Suggested Actions
- [ ] Open WhatsApp Web to read full message
- [ ] Reply to {message['name']}
- [ ] Take necessary action
- [ ] Mark as complete when done

## Notes
*Open WhatsApp Web manually to see full conversation and reply.*
"""

        # Create filename with sender name and timestamp
        safe_name = "".join(c for c in message['name'] if c.isalnum() or c in ' -_').strip()[:30]
        filepath = self.whatsapp_folder / f'WHATSAPP_{safe_name}_{message["id"]}.md'
        filepath.write_text(content, encoding='utf-8')

        # Mark as processed
        self.processed_ids.add(message['chat_id'])

        return filepath


if __name__ == '__main__':
    import sys

    # Default paths for Gold tier
    vault_path = sys.argv[1] if len(sys.argv) > 1 else 'E:/hackathon-0/Gold/AI_Employee_Vault/vault'
    session_path = sys.argv[2] if len(sys.argv) > 2 else 'E:/hackathon-0/Gold/AI_Employee_Vault/whatsapp_session'

    # Check if running in quiet mode (from orchestrator)
    quiet_mode = '--quiet' in sys.argv

    if not quiet_mode:
        print(f"\n{'='*60}")
        print(f"WhatsApp Watcher - Background Mode")
        print(f"{'='*60}")
        print(f"Vault Path: {vault_path}")
        print(f"Session Path: {session_path}")
        print(f"\nHow it works:")
        print(f"   - Opens browser once for QR scan")
        print(f"   - Browser stays open in background (you can minimize it)")
        print(f"   - Checks messages automatically every 30 seconds")
        print(f"   - No need to interact with browser after QR scan")
        print(f"\nDetects ALL unread messages (not just keywords)")
        print(f"Tip: Minimize the browser window to keep it out of sight")
        print(f"Watching for unread messages... (Press Ctrl+C to stop)")
        print(f"{'='*60}\n")
    else:
        print(f"[WHATSAPP] Monitoring: {vault_path}")

    watcher = WhatsAppWatcherSimple(vault_path, session_path)
    watcher.run()
