# MCP Social Media Servers - Setup Guide

## Quick Start

### 1. Copy MCP Config to Qwen Code

**Windows:** Copy to `%APPDATA%\qwen\mcp.json` or `C:\Users\<YourUsername>\AppData\Roaming\qwen\mcp.json`

**Content from project's `mcp-config.json`:**
```json
{
  "mcpServers": {
    "facebook": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/facebook.py"],
      "env": {
        "FACEBOOK_ACCESS_TOKEN": "your_token",
        "FACEBOOK_PAGE_ID": "your_page_id",
        "DRY_RUN": "false"
      }
    },
    "instagram": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram.py"],
      "env": {
        "INSTAGRAM_ACCESS_TOKEN": "your_token",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id",
        "DRY_RUN": "false"
      }
    },
    "twitter": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/twitter.py"],
      "env": {
        "TWITTER_BEARER_TOKEN": "your_token",
        "DRY_RUN": "true"
      }
    }
  }
}
```

### 2. Install Dependencies

```bash
cd AI_Employee_Vault/mcp_servers/social_media
pip install -r requirements.txt
```

### 3. Configure Environment Variables

Add to your `.env` file in project root:

```bash
# Facebook
FACEBOOK_ACCESS_TOKEN=EAAY...
FACEBOOK_PAGE_ID=1069693056219393

# Instagram
INSTAGRAM_ACCESS_TOKEN=EAAY...
INSTAGRAM_ACCOUNT_ID=17841446919185197

# Twitter (optional)
TWITTER_BEARER_TOKEN=your_token

# Global
DRY_RUN=false
```

### 4. Restart Qwen Code

After adding MCP config, restart Qwen Code to load the servers.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Qwen Code (AI Employee)                  │
│  - Reads Business_Goals.md                                   │
│  - Generates platform-specific content                       │
│  - Calls MCP tools for posting                               │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ MCP Protocol (JSON-RPC)
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  MCP Servers (The "Hands")                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │  facebook.py │  │ instagram.py │  │  twitter.py  │       │
│  │              │  │              │  │              │       │
│  │ Tools:       │  │ Tools:       │  │ Tools:       │       │
│  │ - post       │  │ - post       │  │ - post       │       │
│  │ - reply      │  │ - reply      │  │ - analytics  │       │
│  │ - analytics  │  │ - analytics  │  │              │       │
│  └──────────────┘  └──────────────┘  └──────────────┘       │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ HTTP/HTTPS
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    External APIs                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │   Facebook   │  │  Instagram   │  │   Twitter    │       │
│  │  Graph API   │  │  Graph API   │  │   API v2     │       │
│  └──────────────┘  └──────────────┘  └──────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

## Available MCP Tools

### Facebook Server (`facebook.py`)

| Tool | Description | Parameters |
|------|-------------|------------|
| `social_post_facebook` | Post to Facebook page | `text` (required), `media_url` (optional), `link` (optional) |
| `social_reply_facebook_comment` | Reply to a comment | `comment_id`, `message` |
| `social_get_facebook_analytics` | Get page analytics | `days` (default: 7) |

### Instagram Server (`instagram.py`)

| Tool | Description | Parameters |
|------|-------------|------------|
| `social_post_instagram` | Post to Instagram | `caption`, `media_url` (required), `hashtags` (optional) |
| `social_reply_instagram_comment` | Reply to a comment | `comment_id`, `message` |
| `social_get_instagram_analytics` | Get account analytics | `days` (default: 7) |

### Twitter Server (`twitter.py`)

| Tool | Description | Parameters |
|------|-------------|------------|
| `social_post_twitter` | Post to Twitter/X | `text` (required), `media_url` (optional), `reply_to` (optional) |
| `social_get_twitter_analytics` | Get account analytics | `days` (default: 7) |

## Usage Examples

### Via Qwen Code (After MCP Config)

Once MCP servers are configured, Qwen Code can call tools directly:

```
# Post to Facebook
Use social_post_facebook to post: "Hello World!"

# Post to Instagram
Use social_post_instagram with caption "New product!" and media_url "https://..."

# Get analytics
Use social_get_facebook_analytics for last 7 days
```

### Direct Function Call (Testing)

```python
import sys
sys.path.insert(0, 'AI_Employee_Vault/mcp_servers/social_media')

from facebook import post_to_facebook

result = post_to_facebook("Test post from Python!")
print(result)
```

## Workflow: Skill → MCP → Post

```
1. User Request
   ↓
2. Skill generates content → Pending_Approval/
   ↓
3. User approves → move to Approved/
   ↓
4. Qwen calls MCP tool (e.g., social_post_facebook)
   ↓
5. MCP server posts to platform
   ↓
6. Log post → social_media_log.txt
   ↓
7. Move to Posted/
```

## Testing

### Test MCP Server Directly

```bash
# Test Facebook server
cd AI_Employee_Vault/mcp_servers/social_media
python facebook.py

# Send test JSON-RPC request (in another terminal)
echo '{"jsonrpc":"2.0","method":"tools/list","id":1}' | python facebook.py
```

### Test with DRY_RUN

```bash
# Set DRY_RUN=true in .env
DRY_RUN=true

# Run test post
python -c "from facebook import post_to_facebook; print(post_to_facebook('Test'))"
# Should return: {'success': True, 'dry_run': True, ...}
```

## Troubleshooting

### MCP Server Not Loading

**Symptom:** Qwen Code doesn't see MCP tools

**Solutions:**
1. Check MCP config file location is correct
2. Verify JSON syntax in mcp.json
3. Check Python path in config matches your system
4. Restart Qwen Code
5. Check Qwen Code logs for MCP errors

### Facebook Post Fails

**Error:** "Page Access Token expired"

**Solution:**
1. Go to Facebook Graph API Explorer
2. Generate new long-lived token with `pages_manage_posts` permission
3. Update `.env` with new token

**Error:** "Invalid page ID"

**Solution:**
1. Verify PAGE_ID in `.env` matches your Facebook page
2. Page ID is in the URL: `facebook.com/123456789` → ID is `123456789`

### Instagram Post Fails

**Error:** "Media upload failed"

**Solution:**
1. Media URL must be publicly accessible (not localhost)
2. Image must be min 320x320, max 4096x4096
3. Account must be Business or Creator type

### Twitter Post Fails

**Error:** "Authentication failed"

**Solution:**
1. Verify TWITTER_BEARER_TOKEN in `.env`
2. Free tier has limited posting - may need elevated access
3. Check app has write permissions

## Security Best Practices

1. **Never commit `.env`** - Add to `.gitignore`
2. **Use DRY_RUN for testing** - Set `DRY_RUN=true` during development
3. **Rotate tokens monthly** - Update credentials regularly
4. **Minimal permissions** - Request only needed API scopes
5. **Human-in-the-loop** - Always review before posting (approval workflow)

## File Structure

```
AI_Employee_Vault/mcp_servers/social_media/
├── facebook.py          # Facebook MCP server
├── instagram.py         # Instagram MCP server
├── twitter.py           # Twitter MCP server
├── server.py            # Legacy combined server (deprecated)
├── requirements.txt     # Python dependencies
├── README.md            # Full documentation
└── SETUP.md             # This file
```

## References

- [Facebook Graph API](https://developers.facebook.com/docs/graph-api/)
- [Instagram Graph API](https://developers.facebook.com/docs/instagram-api/)
- [Twitter API v2](https://developer.twitter.com/en/docs/twitter-api)
- [MCP Specification](https://modelcontextprotocol.io/)
- [Gold Tier Hackathon Docs](../../../Personal%20AI%20Employee%20Hackathon%200_%20Building%20Autonomous%20FTEs%20in%202026.md)

## Support

For issues or questions:
1. Check README.md for detailed documentation
2. Review hackathon requirements document
3. Test with DRY_RUN=true first
4. Check logs in AI_Employee_Vault/vault/social_media_log.txt
