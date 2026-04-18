# Social Media MCP Servers
## Gold Tier - Multi-Platform Posting

Platform-specific MCP servers for posting to Facebook, Instagram, and Twitter/X.

**Implementation**: Python 3.13+

## Architecture

Each platform has its own dedicated MCP server:

```
AI_Employee_Vault/mcp_servers/social_media/
├── facebook.py            # Facebook MCP server
├── instagram/             # Instagram servers folder
│   ├── instagram.py           # Posting + analytics
│   ├── instagram_comment.py   # Comment replies (microservice)
│   ├── instagram_temporary_server.py  # Local file upload via ngrok
│   └── README.md          # Instagram-specific guide
├── twitter.py             # Twitter/X MCP server
└── extra/                 # Reference documentation
```

## Features

- Platform-specific MCP servers for clean separation
- Post to multiple platforms simultaneously
- Platform-specific content optimization
- Media uploads (images, videos)
- Hashtag and mention handling
- Engagement analytics
- DRY_RUN mode for safe testing
- Comprehensive error handling

## Installation

```bash
cd AI_Employee_Vault/mcp_servers/social_media

# Install Python dependencies
pip install -r requirements.txt
```

## Configuration

Add to your `.env` file:

```bash
# Facebook
FACEBOOK_ACCESS_TOKEN=your_page_access_token
FACEBOOK_PAGE_ID=your_page_id

# Instagram
INSTAGRAM_ACCESS_TOKEN=your_instagram_token
INSTAGRAM_ACCOUNT_ID=your_business_account_id

# Twitter/X
TWITTER_BEARER_TOKEN=your_bearer_token
TWITTER_API_KEY=your_api_key
TWITTER_API_SECRET=your_api_secret
TWITTER_ACCESS_TOKEN=your_access_token
TWITTER_ACCESS_TOKEN_SECRET=your_token_secret

# Global
DRY_RUN=false  # Set to true for testing
```

### Getting API Credentials

#### Facebook
1. Go to [Facebook Developers](https://developers.facebook.com/)
2. Create an app
3. Get Page Access Token with `pages_manage_posts` permission
4. Your Page ID is in the URL of your Facebook page

#### Instagram
1. Convert to Instagram Business account
2. Link to Facebook Page
3. Get Instagram Business Account ID via Graph API Explorer
4. Token needs `instagram_basic` and `instagram_content_publish` permissions

#### Twitter/X
1. Go to [Twitter Developer Portal](https://developer.twitter.com/)
2. Create a project and app
3. Get Bearer Token for OAuth 2.0
4. Note: Posting requires elevated access for some features

## MCP Configuration

Add to Qwen Code MCP config (`%APPDATA%\qwen\mcp.json` on Windows):

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

**Note:** See `mcp-config.json` in the project root for a ready-to-use template.

## Available Tools

### Facebook (`facebook.py`)

#### social_post_facebook
Post to Facebook page.

**Parameters:**
- `text` (required): Post content
- `media_url` (optional): Image or video URL
- `link` (optional): Link to share

**Example:**
```json
{
  "text": "Excited to announce our new product launch!",
  "media_url": "https://example.com/image.jpg",
  "link": "https://example.com/product"
}
```

#### social_reply_facebook_comment
Reply to a Facebook comment.

**Parameters:**
- `comment_id` (required): Facebook comment ID to reply to
- `message` (required): Reply message content

#### social_get_facebook_analytics
Get Facebook page analytics.

**Parameters:**
- `days` (optional): Days to look back (default: 7)

### Instagram (`instagram.py`)

#### social_post_instagram
Post to Instagram.

**Parameters:**
- `caption` (required): Post caption
- `media_url` (required): Image URL
- `hashtags` (optional): Array of hashtags

**Example:**
```json
{
  "caption": "Beautiful sunset today!",
  "media_url": "https://example.com/sunset.jpg",
  "hashtags": ["sunset", "photography", "nature"]
}
```

#### social_reply_instagram_comment
Reply to an Instagram comment.

**Parameters:**
- `comment_id` (required): Instagram comment ID
- `message` (required): Reply message content

#### social_get_instagram_analytics
Get Instagram account analytics.

**Parameters:**
- `days` (optional): Days to look back (default: 7)

### Twitter (`twitter.py`)

#### social_post_twitter
Post to Twitter/X.

**Parameters:**
- `text` (required): Tweet content (max 280 characters)
- `media_url` (optional): Image or video URL
- `reply_to` (optional): Tweet ID to reply to

**Example:**
```json
{
  "text": "Just launched our new feature! Check it out.",
  "media_url": "https://example.com/feature.png"
}
```

#### social_get_twitter_analytics
Get Twitter account analytics.

**Parameters:**
- `days` (optional): Days to look back (default: 7)

## Usage with Qwen Code

Once MCP servers are configured, use in Qwen Code:

```
# Post to Facebook
Use the social_post_facebook tool to post: "Hello World!"

# Post to Instagram
Use social_post_instagram with caption "New product!" and media_url "https://..."

# Post to Twitter
Use social_post_twitter to tweet: "Exciting news!"

# Get analytics
Use social_get_facebook_analytics to check performance last 7 days
```

## Response Format

All tools return JSON responses:

**Success:**
```json
{
  "success": true,
  "post_id": "123456789",
  "url": "https://facebook.com/123456789"
}
```

**Error:**
```json
{
  "success": false,
  "error": "Facebook credentials not configured"
}
```

## Platform Limits

| Platform | Text Limit | Media | Rate Limit |
|----------|-----------|-------|------------|
| Facebook | 63,206 chars | Images, Videos | 200 calls/hour |
| Instagram | 2,200 chars | Images only | 200 calls/hour |
| Twitter | 280 chars | Images, GIFs | 50 tweets/day (free) |

## Error Handling

The servers handle common errors:

- **Authentication errors**: Returns clear message to check credentials
- **Rate limits**: Logs warning, suggests retry time
- **Media errors**: Validates URLs before upload
- **Network errors**: Includes timeout handling

## Security Best Practices

1. **Never commit credentials**: Use `.env` file, add to `.gitignore`
2. **Use DRY_RUN for testing**: Set `DRY_RUN=true` during development
3. **Rotate tokens monthly**: Update credentials regularly
4. **Minimal permissions**: Request only needed API permissions
5. **Review before posting**: Always review content before execution

## Troubleshooting

### Facebook: "Page Access Token expired"
- Generate new long-lived token via Facebook Graph API Explorer
- Ensure app has `pages_manage_posts` permission

### Instagram: "Media upload failed"
- Media URL must be publicly accessible
- Image must meet size requirements (min 320x320, max 4096x4096)
- Account must be Business or Creator

### Twitter: "Authentication failed"
- Check Bearer Token is correct
- Verify app has write permissions
- Free tier has limited posting capabilities

## Testing

```bash
# Test with DRY_RUN first
export DRY_RUN=true
python facebook.py

# Test individual functions
python -c "
from facebook import post_to_facebook
result = post_to_facebook('Test post')
print(result)
"
```

## References

- [Facebook Graph API](https://developers.facebook.com/docs/graph-api/)
- [Instagram Graph API](https://developers.facebook.com/docs/instagram-api/)
- [Twitter API v2](https://developer.twitter.com/en/docs/twitter-api)
- [MCP Specification](https://modelcontextprotocol.io/)

## License

MIT License
