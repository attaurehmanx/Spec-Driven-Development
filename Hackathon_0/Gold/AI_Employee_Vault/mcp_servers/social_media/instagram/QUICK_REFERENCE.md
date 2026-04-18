# Instagram MCP Servers - Quick Reference

## 📁 Files

| File | Purpose | Tools |
|------|---------|-------|
| `instagram.py` | Post images + analytics | `social_post_instagram`, `social_get_instagram_analytics` |
| `instagram_comment.py` | Comment replies | `social_reply_instagram_comment`, `social_get_instagram_comments` |
| `instagram_temporary_server.py` | Local file uploads | `instagram_post_local_temporary`, `instagram_post_url` |

---

## 🚀 Quick Commands

### Post from URL
```python
social_post_instagram(
    caption="Hello Instagram! 📸",
    media_url="https://example.com/image.jpg",
    hashtags=["hello", "instagram"]
)
```

### Post Local File
```python
instagram_post_local_temporary(
    caption="Local file post! ✨",
    media_path="E:/hackathon-0/Gold/image.png",
    hashtags=["local", "test"]
)
```

### Reply to Comment
```python
social_reply_instagram_comment(
    comment_id="17891269263305259",
    message="Thanks! 🙏"
)
```

### Get Analytics
```python
social_get_instagram_analytics(days=7)
```

---

## 🔑 Required .env Variables

```bash
# Instagram
INSTAGRAM_ACCESS_TOKEN=your_token_here
INSTAGRAM_ACCOUNT_ID=your_account_id_here

# Facebook (required for Instagram Business)
FACEBOOK_ACCESS_TOKEN=your_facebook_token
FACEBOOK_PAGE_ID=your_page_id

# ngrok (for local file uploads)
NGROK_AUTH_TOKEN=your_ngrok_token

# Global
DRY_RUN=false  # true for testing
```

---

## 📊 MCP Config Paths

```json
{
  "instagram": {
    "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram.py"]
  },
  "instagram-comment": {
    "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_comment.py"]
  },
  "instagram-temporary": {
    "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_temporary_server.py"]
  }
}
```

---

## ✅ Supported Image Formats

- `.jpg`
- `.jpeg`
- `.png`

---

## 🔗 Full Documentation

See [README.md](README.md) for complete guide.
