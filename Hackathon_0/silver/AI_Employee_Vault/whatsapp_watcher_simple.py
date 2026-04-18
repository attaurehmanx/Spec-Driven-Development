"""
WhatsApp Watcher - Simple Version
Uses page-wide text search instead of specific selectors
"""
from pathlib import Path
from datetime import datetime
from base_watcher import BaseWatcher
import json
import time
import re

try:
    from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeout
    PLAYWRIGHT_AVAILABLE = True
except ImportError:
    PLAYWRIGHT_AVAILABLE = False
    print("WARNING: Playwright not installed. Run: pip install playwright")

class WhatsAppWatcherSimple(BaseWatcher):
    """WhatsApp watcher with simple page-wide text search"""

    def __init__(self, vault_path: str, session_path: str = './whatsapp_session', check_interval: int = 30):
        super().__init__(vault_path, check_interval, watcher_type='whatsapp')
        self.session_path = Path(session_path)
        self.session_path.mkdir(parents=True, exist_ok=True)
        self.processed_messages = set()
        self.last_check_content = ""

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
            return []

        messages = []

        try:
            # Don't reload - just check the current page state
            time.sleep(1)  # Brief wait

            # Get the main chat list container
            try:
                # Try to find the main sidebar/chat list area
                main_selectors = [
                    '#pane-side',
                    '[data-testid="chat-list"]',
                    'div[aria-label="Chat list"]',
                    'div[role="grid"]',
                    'div[role="list"]'
                ]

                main_container = None
                for sel in main_selectors:
                    main_container = self.page.query_selector(sel)
                    if main_container:
                        break

                if not main_container:
                    return []

                # Now find all chat items within the container
                chat_selectors = [
                    'div[role="row"]',
                    'div[tabindex="-1"]',
                    'div > div > div',
                    'div[class]'
                ]

                chat_containers = []
                for sel in chat_selectors:
                    containers = main_container.query_selector_all(sel)
                    if containers and len(containers) > 0:
                        # Filter to only elements with substantial text
                        chat_containers = [c for c in containers if len(c.inner_text()) > 10]
                        if len(chat_containers) > 0:
                            break

                if not chat_containers:
                    return []

            except Exception as e:
                return []

            all_unread_chats = []

            # Parse individual chat containers for unread messages
            for idx, container in enumerate(chat_containers[:50]):  # Check first 50 chats
                try:
                    # Get all attributes and text
                    text = container.inner_text()
                    aria_label = container.get_attribute('aria-label') or ""

                    # Check if this chat has unread indicator
                    is_unread = False

                    # Method 1: Check aria-label for "unread"
                    if 'unread' in aria_label.lower():
                        is_unread = True

                    # Method 2: Check for unread badge/counter elements
                    if not is_unread:
                        unread_selectors = [
                            '[data-testid="icon-unread-count"]',
                            'span[data-icon="unread-count"]',
                            'span[aria-label*="unread"]',
                            'div[aria-label*="unread"]',
                            'span[class*="unread"]'
                        ]
                        for sel in unread_selectors:
                            unread_badge = container.query_selector(sel)
                            if unread_badge:
                                is_unread = True
                                break

                    # Method 3: Check for visual unread indicators in text
                    if not is_unread and text:
                        lines = text.split('\n')
                        for line in lines:
                            line = line.strip()
                            # Check if line is just a number (unread count)
                            if line.isdigit() and int(line) > 0:
                                is_unread = True
                                break

                    # Method 4: Check for bold/unread styling
                    if not is_unread:
                        bold_elem = container.query_selector('span[class*="unread"], div[class*="unread"], strong, span[style*="font-weight"]')
                        if bold_elem:
                            is_unread = True

                    if is_unread:
                        # Parse chat info
                        lines = [line.strip() for line in text.split('\n') if line.strip()]

                        if len(lines) >= 1:
                            # First line is usually the chat name
                            chat_name = lines[0] if lines[0] else "Unknown"

                            # Rest is message text
                            message_text = ' '.join(lines[1:]) if len(lines) > 1 else "New message"

                            # Try to find time (usually has : like "22:12")
                            time_str = "Unknown"
                            for line in lines:
                                # Match time patterns like "22:12" or "10:30 AM"
                                if ':' in line and len(line) < 15 and any(c.isdigit() for c in line):
                                    time_str = line
                                    break

                            # Create unique ID for this message
                            message_id = f"{chat_name}_{time_str}_{len(message_text)}"

                            if message_id not in self.processed_messages:
                                all_unread_chats.append({
                                    'name': chat_name,
                                    'time': time_str,
                                    'message': message_text[:200],  # First 200 chars
                                    'message_id': message_id
                                })
                                self.processed_messages.add(message_id)
                except Exception as e:
                    continue

            # If we found unread chats, create a message
            if all_unread_chats:
                message_id = f"unread_{datetime.now().strftime('%Y%m%d%H%M%S')}"
                messages.append({
                    'id': message_id,
                    'chats': all_unread_chats,
                    'timestamp': datetime.now().isoformat()
                })

        except Exception as e:
            import traceback
            traceback.print_exc()
            self.logger.error(f"Error checking WhatsApp: {e}")
            # If there's an error, try to reinitialize the browser
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
        """Create action file for WhatsApp unread messages"""
        timestamp = datetime.fromisoformat(message['timestamp'])

        # New format with all unread chats
        chats = message['chats']

        content = f"""---
type: whatsapp_message
received: {message['timestamp']}
total_unread_chats: {len(chats)}
priority: normal
status: pending
---

## Unread WhatsApp Messages

**Total Unread Chats**: {len(chats)}
**Date**: {timestamp.strftime('%Y-%m-%d')}
**Time**: {timestamp.strftime('%H:%M:%S')}
**Status**: Pending

---

"""
        # Add each chat as a separate section
        for i, chat in enumerate(chats, 1):
            content += f"""### Chat {i}: {chat['name']}

**Name**: {chat['name']}

**Time**: {chat['time']}

**Date**: {timestamp.strftime('%Y-%m-%d')}

**Message Preview**: {chat['message']}

**Status**: Unread

---

"""

        # Add common footer
        content += """## Suggested Actions
- [ ] Open WhatsApp Web to read full messages
- [ ] Reply to each sender
- [ ] Take necessary actions
- [ ] Mark as complete when done

## Notes
*Open WhatsApp Web manually to see full conversations and reply.*
"""

        filepath = self.needs_action / f'WHATSAPP_{message["id"]}.md'
        filepath.write_text(content, encoding='utf-8')

        return filepath


if __name__ == '__main__':
    import sys

    vault_path = sys.argv[1] if len(sys.argv) > 1 else './vault'
    session_path = sys.argv[2] if len(sys.argv) > 2 else './whatsapp_session'

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
