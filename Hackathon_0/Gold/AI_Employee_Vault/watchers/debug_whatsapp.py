"""
Debug script to test WhatsApp Web selectors
"""
from playwright.sync_api import sync_playwright
import time

print("Launching browser...")

try:
    playwright = sync_playwright().start()

    # Launch browser with persistent context
    browser = playwright.chromium.launch_persistent_context(
        'E:/hackathon-0/Gold/AI_Employee_Vault/whatsapp_session',
        headless=False,
        channel='msedge',
        args=['--no-sandbox', '--disable-blink-features=AutomationControlled']
    )

    page = browser.pages[0] if browser.pages else browser.new_page()
    
    print("Going to WhatsApp Web...")
    page.goto('https://web.whatsapp.com', timeout=90000)
    
    print("\n=== WAITING FOR WHATSAPP TO LOAD ===")
    print("Scan QR code if needed. Waiting 60 seconds...")
    
    # Wait for WhatsApp to load
    for i in range(12):
        time.sleep(5)
        pane = page.query_selector('#pane-side')
        if pane:
            print(f"✅ WhatsApp loaded at iteration {i+1}!")
            break
        print(f"  Waiting... ({(i+1)*5}s)")
    else:
        print("❌ WhatsApp did not load in 60 seconds")
    
    print("\n=== Testing Selectors ===\n")

    # Check page title
    try:
        title = page.title()
        print(f"Page title: {title}")
    except Exception as e:
        print(f"Error getting title: {e}")

    # Check for pane-side
    pane_side = page.query_selector('#pane-side')
    if pane_side:
        print("✅ Found #pane-side")
        
        # Try different selectors
        selectors = [
            'li[role="row"]',
            'div[tabindex="0"]',
            'div[role="row"]',
            'div[class*="unread"]',
            '[data-testid="icon-unread-count"]',
            '[aria-label*="unread"]'
        ]
        
        for sel in selectors:
            try:
                items = pane_side.query_selector_all(sel)
                if items:
                    print(f"  ✅ Found {len(items)} items with: {sel}")
                    # Print text of first few
                    for i, item in enumerate(items[:3]):
                        try:
                            text = item.inner_text()[:100].replace('\n', ' | ')
                            aria = item.get_attribute('aria-label') or ""
                            print(f"     [{i}] Text: {text}")
                            print(f"         Aria: {aria[:50]}")
                        except Exception as e:
                            print(f"     [{i}] Error: {e}")
                else:
                    print(f"  ❌ No items with: {sel}")
            except Exception as e:
                print(f"  ❌ Error with {sel}: {e}")
    else:
        print("❌ #pane-side not found")

    print("\n=== Press Enter to close browser ===")
    input()
    browser.close()
    playwright.stop()
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
