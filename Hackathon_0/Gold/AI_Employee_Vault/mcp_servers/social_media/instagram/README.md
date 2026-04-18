# Instagram MCP Servers - Complete Guide
## Gold Tier - Instagram Posting & Engagement

**Location**: `AI_Employee_Vault/mcp_servers/social_media/instagram/`

---

## 📁 File Structure

```
instagram/
├── instagram.py                      # Main posting server (posts + analytics)
├── instagram_comment.py              # Comment reply microservice
├── instagram_temporary_server.py     # Local file upload via ngrok
├── README.md                         # This guide
└── requirements.txt                  # Python dependencies (shared)
```

---

## 🎯 Overview

Three dedicated MCP servers for Instagram operations:

| Server | Purpose | Tools |
|--------|---------|-------|
| **instagram.py** | Posting images + analytics | `social_post_instagram`, `social_get_instagram_analytics` |
| **instagram-comment.py** | Comment replies | `social_reply_instagram_comment`, `social_get_instagram_comments` |
| **instagram-temporary-server.py** | Local file uploads | `instagram_post_local_temporary`, `instagram_post_url` |

---

## 🚀 Quick Start

### Step 1: Configure Credentials

Add to your `.env` file (project root: `E:\hackathon-0\Gold\.env`):

```bash
# Instagram Business Account
INSTAGRAM_ACCESS_TOKEN=EAAYkwE09etEBQZCnLXQTkrWigNEYconZBUflUAHZBZCskprqeK0UC8LCW7p4WQ0uhtMwB6xtZCZCU0zLT6H8IGpOEZBY605ZBE8C51ctBN7mFp482XYPtDSFX4GTbNqRWf1ZAPHtGn9WTgschpm7PbPc8qAWbgSqqRiMsfYfbVZBgrZCDLY9wAWHPupakwaJUKgOg2bwFtr
INSTAGRAM_ACCOUNT_ID=17841446919185197

# Facebook (required for Instagram Business)
FACEBOOK_ACCESS_TOKEN=EAAYkwE09etEBQZCnLXQTkrWigNEYconZBUflUAHZBZCskprqeK0UC8LCW7p4WQ0uhtMwB6xtZCZCU0zLT6H8IGpOEZBY605ZBE8C51ctBN7mFp482XYPtDSFX4GTbNqRWf1ZAPHtGn9WTgschpm7PbPc8qAWbgSqqRiMsfYfbVZBgrZCDLY9wAWHPupakwaJUKgOg2bwFtr
FACEBOOK_PAGE_ID=1069693056219393

# ngrok (for local file uploads)
NGROK_AUTH_TOKEN=your_ngrok_token_here

# Global
DRY_RUN=false
```

### Step 2: Add MCP Servers to Qwen Code

**Windows:** Copy to `%APPDATA%\qwen\mcp.json`

```json
{
  "mcpServers": {
    "instagram": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram.py"],
      "env": {
        "INSTAGRAM_ACCESS_TOKEN": "your_token_here",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id_here",
        "DRY_RUN": "false"
      }
    },
    "instagram-comment": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_comment.py"],
      "env": {
        "INSTAGRAM_ACCESS_TOKEN": "your_token_here",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id_here",
        "DRY_RUN": "false"
      }
    },
    "instagram-temporary": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_temporary_server.py"],
      "env": {
        "NGROK_AUTH_TOKEN": "your_ngrok_token_here",
        "INSTAGRAM_ACCESS_TOKEN": "your_token_here",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id_here",
        "DRY_RUN": "false"
      }
    }
  }
}
```

### Step 3: Install Dependencies

```bash
cd E:/hackathon-0/Gold
pip install -r requirements.txt
```

Required packages:
- `requests`
- `python-dotenv`
- `pyngrok` (for local file uploads)

---

## 📸 Server 1: instagram.py (Main Posting)

### Purpose
Post images to Instagram from URLs and get analytics.

### Tools

#### `social_post_instagram`
Post an image to Instagram.

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `caption` | string | ✅ Yes | Post caption text |
| `media_url` | string | ⚠️ Conditional | Public image URL |
| `media_path` | string | ⚠️ Conditional | Local file path (not supported - use instagram-temporary) |
| `hashtags` | array | ❌ No | Array of hashtag strings |

**Example Usage:**
```python
# Via MCP tool call
social_post_instagram(
    caption="Beautiful sunset today! 🌅",
    media_url="https://example.com/sunset.jpg",
    hashtags=["sunset", "photography", "nature"]
)
```

**Response:**
```json
{
  "success": true,
  "post_id": "18107209900658133",
  "url": "https://instagram.com/p/18107209900658133"
}
```

---

#### `social_get_instagram_analytics`
Get engagement metrics for your Instagram account.

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `days` | number | 7 | Number of days to look back |

**Example Usage:**
```python
social_get_instagram_analytics(days=7)
```

**Response:**
```json
{
  "success": true,
  "platform": "instagram",
  "period_days": 7,
  "data": [
    {"name": "impressions", "values": [{"value": 1250}]},
    {"name": "reach", "values": [{"value": 980}]},
    {"name": "engagement", "values": [{"value": 156}]}
  ]
}
```

---

## 💬 Server 2: instagram-comment.py (Comment Replies)

### Purpose
Reply to comments on your Instagram posts and fetch comments.

### Tools

#### `social_reply_instagram_comment`
Reply to a specific comment on your post.

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `comment_id` | string | ✅ Yes | Instagram comment ID |
| `message` | string | ✅ Yes | Reply message text |

**Example Usage:**
```python
social_reply_instagram_comment(
    comment_id="17891269263305259",
    message="Thank you for your kind words! 🙏"
)
```

**Response:**
```json
{
  "success": true,
  "reply_id": "18010927697840544",
  "url": "https://instagram.com/18010927697840544"
}
```

**⚠️ Important:** Use `/replies` endpoint (not `/comments`) for replying to comments.

---

#### `social_get_instagram_comments`
Fetch comments from a specific post.

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `media_id` | string | ✅ Yes | Instagram post/media ID |
| `limit` | number | 20 | Max comments to fetch |

**Example Usage:**
```python
social_get_instagram_comments(
    media_id="18107209900658133",
    limit=50
)
```

**Response:**
```json
{
  "success": true,
  "media_id": "18107209900658133",
  "comments": [
    {
      "id": "17891269263305259",
      "text": "Amazing photo! 📸",
      "username": "user123",
      "timestamp": "2026-03-16T19:45:00+0000",
      "like_count": 5
    }
  ],
  "total_count": 1
}
```

---

## 🖼️ Server 3: instagram-temporary-server.py (Local Files)

### Purpose
Post local image files to Instagram without third-party hosting.

### How It Works
```
1. Start local HTTP server (port 8080)
2. Create ngrok tunnel (temporary public URL)
3. Instagram fetches image from ngrok URL
4. Close server and tunnel
5. Result: Image on Instagram, NO permanent hosting!
```

### Tools

#### `instagram_post_local_temporary`
Post a local image file to Instagram.

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `caption` | string | ✅ Yes | Post caption |
| `media_path` | string | ✅ Yes | Local file path (`.jpg`, `.jpeg`, `.png`) |
| `hashtags` | array | ❌ No | Array of hashtags |

**Example Usage:**
```python
instagram_post_local_temporary(
    caption="Eid Mubarak! ✨🌙",
    media_path="E:/hackathon-0/Gold/eid_mubarak_temp.png",
    hashtags=["EidMubarak", "Eid2026", "Celebration"]
)
```

**Workflow:**
1. ✅ Validates file exists
2. ✅ Starts local HTTP server
3. ✅ Creates ngrok tunnel
4. ✅ Posts to Instagram
5. ✅ Closes server & tunnel

**Response:**
```json
{
  "success": true,
  "instagram_post_id": "18107209900658133",
  "instagram_url": "https://instagram.com/p/18107209900658133",
  "upload_method": "local_file_temporary_server",
  "message": "Posted to Instagram (temporary server closed, no permanent hosting)"
}
```

---

#### `instagram_post_url`
Post from a public image URL.

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `caption` | string | ✅ Yes | Post caption |
| `media_url` | string | ✅ Yes | Public image URL |
| `hashtags` | array | ❌ No | Array of hashtags |

**Example Usage:**
```python
instagram_post_url(
    caption="Check out our new product! 🚀",
    media_url="https://example.com/product.jpg",
    hashtags=["product", "launch", "innovation"]
)
```

---

## 📋 Usage Examples

### Example 1: Post from URL (instagram.py)

```python
# Using /social-poster skill
/social-poster --platform instagram "Beautiful sunset!" --media-url "https://example.com/sunset.jpg"
```

### Example 2: Post Local File (instagram-temporary)

```python
# Using /social-poster skill
/social-poster --platform instagram "Eid Mubarak!" --media-path "E:/hackathon-0/Gold/eid_mubarak_temp.png"
```

### Example 3: Reply to Comment (instagram-comment)

```python
# Via MCP tool call
social_reply_instagram_comment(
    comment_id="17891269263305259",
    message="Thanks for the love! ❤️"
)
```

### Example 4: Get Analytics (instagram.py)

```python
# Get last 7 days of analytics
social_get_instagram_analytics(days=7)
```

### Example 5: Fetch Comments (instagram-comment)

```python
# Get comments from a post
social_get_instagram_comments(
    media_id="18107209900658133",
    limit=100
)
```

---

## 🔧 Troubleshooting

### Issue: "Instagram credentials not configured"

**Solution:** Check `.env` file has correct values:
```bash
INSTAGRAM_ACCESS_TOKEN=your_token_here
INSTAGRAM_ACCOUNT_ID=your_account_id_here
```

---

### Issue: "File not found" (local upload)

**Solution:** Verify file path is absolute and file exists:
```python
# ✅ Correct
media_path="E:/hackathon-0/Gold/image.png"

# ❌ Wrong (relative path)
media_path="image.png"
```

---

### Issue: "400 Bad Request" (comment reply)

**Cause:** Using wrong API endpoint

**Solution:** Ensure using `/{comment_id}/replies` (not `/comments`)
- Fixed in `instagram_comment.py` ✅

---

### Issue: ngrok tunnel fails

**Solutions:**
1. Check ngrok auth token in `.env`:
   ```bash
   NGROK_AUTH_TOKEN=your_token_here
   ```
2. Get token from: https://dashboard.ngrok.com/get-started/your-authtoken
3. Install pyngrok: `pip install pyngrok`

---

### Issue: "Image format not supported"

**Supported formats:** `.jpg`, `.jpeg`, `.png`

**Solution:** Convert image to supported format.

---

## 📊 Best Practices

### Posting Frequency
| Account Type | Recommended Posts/Day |
|--------------|----------------------|
| Business | 1-2 posts |
| Creator | 2-3 posts |
| Personal | 1 post |

### Optimal Posting Times
- **Morning:** 6-9 AM
- **Lunch:** 11 AM-1 PM
- **Evening:** 6-9 PM (best engagement)

### Caption Guidelines
- **Length:** 50-150 words (optimal engagement)
- **Hashtags:** 10-15 relevant tags
- **Emojis:** 2-5 per post
- **Call-to-Action:** Always include one

### Hashtag Strategy
```
#BrandHashtag          (1-2)
#IndustryHashtag      (3-5)
#NicheHashtag         (5-7)
#TrendingHashtag     (1-2)
```

---

## 🔒 Security

### Token Management
- ✅ Store tokens in `.env` (never commit to git)
- ✅ Rotate tokens every 90 days
- ✅ Use DRY_RUN=true for testing
- ✅ Limit token permissions to minimum required

### DRY_RUN Mode
```bash
# Testing - no real posts
DRY_RUN=true

# Production - real posts
DRY_RUN=false
```

---

## 📈 Analytics Metrics

| Metric | Description | Good Benchmark |
|--------|-------------|----------------|
| **Impressions** | Total views | 1000+ per post |
| **Reach** | Unique viewers | 800+ per post |
| **Engagement** | Likes + comments + saves | 50+ per post |
| **Engagement Rate** | Engagement / Reach | 3-6% |
| **Follower Growth** | New followers | +50/month |

---

## 🎯 API Reference

### Instagram Graph API Endpoints

| Action | Endpoint | Method |
|--------|----------|--------|
| Create Media Container | `/{ig_user_id}/media` | POST |
| Publish Media | `/{ig_user_id}/media_publish` | POST |
| Reply to Comment | `/{comment_id}/replies` | POST |
| Get Comments | `/{media_id}/comments` | GET |
| Get Insights | `/{ig_user_id}/insights` | GET |

### Required Permissions
- `instagram_basic`
- `instagram_content_publish`
- `pages_read_engagement`

---

## 📚 Resources

### Official Documentation
- [Instagram Graph API](https://developers.facebook.com/docs/instagram-api)
- [Instagram Content Publishing](https://developers.facebook.com/docs/instagram-api/guides/content-publishing)
- [Facebook Graph API Explorer](https://developers.facebook.com/tools/explorer/)

### Tools
- [ngrok](https://ngrok.com/) - Temporary public URLs
- [Meta Business Suite](https://business.facebook.com/) - Manage Instagram

---

## 📝 Changelog

### v2.0 (2026-03-16)
- ✅ Split comment replies to separate microservice
- ✅ Fixed comment reply endpoint (`/replies` instead of `/comments`)
- ✅ Added `social_get_instagram_comments` tool
- ✅ Organized Instagram servers into dedicated folder

### v1.0 (2026-03-13)
- ✅ Initial Instagram MCP server
- ✅ Local file upload via ngrok
- ✅ Analytics support

---

## 🆘 Support

For issues or questions:
1. Check this guide's [Troubleshooting](#troubleshooting) section
2. Review server logs for error messages
3. Verify credentials in `.env`
4. Test with DRY_RUN=true first

---

*Last Updated: 2026-03-16*
*Version: 2.0*
