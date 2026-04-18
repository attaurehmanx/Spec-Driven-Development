# Browser MCP Server
# Gold Tier - Web Automation

MCP server for browser automation using Playwright.

## Features

- Navigate to URLs
- Fill forms
- Click buttons
- Extract data
- Take screenshots
- Handle authentication
- Execute JavaScript

## Installation

```bash
cd AI_Employee_Vault/mcp_servers/browser
npm install
playwright install chromium
```

## Configuration

Add to Claude Code MCP config:

```json
{
  "servers": [
    {
      "name": "browser",
      "command": "node",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/browser/index.js"],
      "env": {
        "HEADLESS": "true"
      }
    }
  ]
}
```

## Available Tools

### browser_navigate
Navigate to a URL.

**Parameters:**
- `url`: URL to navigate to
- `wait_for` (optional): Selector to wait for

### browser_click
Click an element.

**Parameters:**
- `selector`: CSS selector
- `wait_for` (optional): Wait for element

### browser_fill
Fill a form field.

**Parameters:**
- `selector`: CSS selector
- `value`: Value to fill

### browser_extract
Extract text from page.

**Parameters:**
- `selector`: CSS selector
- `attribute` (optional): Attribute to extract

### browser_screenshot
Take a screenshot.

**Parameters:**
- `path`: Save path
- `full_page` (optional): Full page screenshot

### browser_execute
Execute JavaScript.

**Parameters:**
- `script`: JavaScript code to execute

## Use Cases

### Payment Portal Automation
```json
{
  "steps": [
    {"action": "navigate", "url": "https://payment-portal.com"},
    {"action": "fill", "selector": "#username", "value": "user"},
    {"action": "fill", "selector": "#password", "value": "pass"},
    {"action": "click", "selector": "#login"},
    {"action": "wait", "selector": "#dashboard"},
    {"action": "screenshot", "path": "dashboard.png"}
  ]
}
```

### Data Extraction
```json
{
  "url": "https://example.com/data",
  "extract": [
    {"selector": ".price", "name": "price"},
    {"selector": ".title", "name": "title"}
  ]
}
```

## Security

- All actions require approval
- Sandboxed browser context
- No persistent cookies (unless specified)
- Screenshot all sensitive actions
- Comprehensive logging

## Testing

```bash
# Test navigation
node test_navigate.js

# Test form filling
node test_form.js

# Test extraction
node test_extract.js
```

## Implementation Notes

This is a template MCP server. Full implementation requires:

1. **Playwright:**
   ```bash
   npm install playwright
   ```

2. **Browser binaries** installed

3. **Session management** for authenticated sites

## References

- [Playwright Documentation](https://playwright.dev/)
- [MCP Browser Use](https://github.com/anthropics/mcp-browser-use)
