# Social Poster Skill - Quick Reference

## Updated for Instagram Local File Upload Support

**Last Updated:** 2026-03-15  
**Version:** 3.0

---

## Quick Usage

### **Instagram with Local File**
```bash
/social-poster --platform instagram "Your caption" --media-path "C:/path/to/image.jpg"
```

### **Instagram with URL**
```bash
/social-poster --platform instagram "Your caption" --media-url "https://example.com/image.jpg"
```

### **Facebook with Local File**
```bash
/social-poster --platform facebook "Your message" --media-path "C:/path/to/image.jpg"
```

### **Twitter**
```bash
/social-poster --platform twitter "Your tweet text"
```

---

## MCP Tools by Platform

| Platform | Tool Name | Parameters | Use Case |
|----------|-----------|------------|----------|
| **Instagram (Local)** | `instagram_post_local_temporary` | `caption`, `media_path`, `hashtags` | Upload from computer (temporary server, NO third-party!) |
| **Instagram (Local)** | `instagram_post_local` | `caption`, `media_path`, `hashtags` | Upload from computer (Facebook→Instagram) |
| **Instagram (URL)** | `instagram_post_url` | `caption`, `media_url`, `hashtags` | Post from web URL |
| **Facebook** | `facebook_post_local` | `message`, `media_path` | Upload from computer |
| **Twitter** | `social_post_twitter` | `text`, `media_url` | Post tweet |

---

## Workflow

```
1. User Request
   ↓
2. Generate Content (caption + hashtags)
   ↓
3. Save to Pending_Approval/
   ↓
4. Show Preview + AskUserQuestion
   ↓
5. User Decision:
   - Approve and Post → Call MCP tool → Log → Posted/
   - Approve Only → Approved/
   - Request Changes → Regenerate → Back to step 2
   - Reject → Rejected/
```

---

## Instagram Posting: Which Tool?

| Scenario | Use This Tool | Why |
|----------|---------------|-----|
| Image on computer | `instagram_post_local_temporary` | ✅ BEST - Temporary server, NO third-party, Instagram ONLY |
| Image on computer (alt) | `instagram_post_local` | Facebook→Instagram workflow |
| Image on website | `instagram_post_url` | Direct URL posting |
| Already have URL | `instagram_post_url` or `social_post_instagram` | Direct URL posting |
| Want Facebook first | `instagram_post_local` (does this automatically) |

---

## Example: Local File Upload (Temporary Server)

**User Input:**
```bash
/social-poster --platform instagram "Ramadan Mubarak!" --media-path "E:/hackathon-0/Gold/ramadan_mubarak.jpg"
```

**What Claude Does:**
1. Generates caption: "Ramadan Mubarak! 🌙✨ Wishing everyone a blessed month..."
2. Generates hashtags: `['RamadanMubarak', 'BlessedMonth', 'Ramadan2026']`
3. Saves to `Pending_Approval/instagram_ramadan_20260315.md`
4. Shows preview with AskUserQuestion
5. User approves
6. Calls: `instagram_post_local_temporary(caption, media_path, hashtags)`
7. MCP server:
   - Starts temporary HTTP server on your computer (port 8080)
   - Creates ngrok tunnel: `https://abc123.ngrok-free.app/ramadan_mubarak.jpg`
   - Instagram fetches image from tunnel
   - Creates Instagram media container
   - Publishes to Instagram
   - CLOSES server and tunnel
   - Tunnel URL no longer works
8. Logs result to `social_media_log.txt`
9. Moves file to `Posted/instagram/`
10. Returns success with URL

**Result:**
- ✅ Posted to Instagram
- ❌ NO Facebook post
- ❌ NO permanent hosting
- ❌ NO third-party service

---

## Error Handling

| Error | Action |
|-------|--------|
| File not found | Tell user: "File not found at {path}. Please check the path." |
| Invalid format | Tell user: "Unsupported format. Use .jpg, .jpeg, or .png" |
| Credentials error | Tell user: "API credentials not configured. Check .env file." |
| MCP not available | Save to Approved/, tell user to post manually |

---

## Logging Format

```txt
---
Posted: 2026-03-15T12:00:00Z
Platform: Instagram
Account: AI Employee
Post ID: 18112591996662663
URL: https://instagram.com/p/18112591996662663
Topic: Ramadan Mubarak
Media Path: E:/hackathon-0/Gold/ramadan_mubarak.jpg
Upload Method: local_file_temporary_server
Status: Success
Note: Temporary server used - no permanent hosting
---
```

---

## Best Practices

1. **Always use AskUserQuestion** before posting
2. **Show full preview** including image path
3. **Log every post** to social_media_log.txt
4. **Check for duplicates** before generating
5. **Use instagram_post_local** for local files (not instagram_post_url)
6. **Move files** to correct folders based on user decision
7. **Include hashtags** in Instagram posts (10-15 recommended)

---

## Supported Platforms

| Platform | Local Files | URLs | MCP Tool |
|----------|-------------|------|----------|
| Instagram | ✅ YES (2 methods) | ✅ YES | `instagram_post_local_temporary` (temporary server), `instagram_post_local` (Facebook), `instagram_post_url` |
| Facebook | ✅ YES | ✅ YES | `facebook_post_local` |
| Twitter | ⚠️ (via URL) | ✅ YES | `social_post_twitter` |
| LinkedIn | ❌ NO | ❌ NO | Browser automation (Playwright) |

---

## Configuration Required

Add to `%APPDATA%\qwen\mcp.json`:

```json
{
  "mcpServers": {
    "instagram-temporary": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram_temporary_server.py"],
      "env": {
        "NGROK_AUTH_TOKEN": "your_ngrok_token",
        "INSTAGRAM_ACCESS_TOKEN": "your_instagram_token",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id",
        "DRY_RUN": "false"
      }
    },
    "instagram-local": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram_local.py"],
      "env": {
        "FACEBOOK_ACCESS_TOKEN": "your_token",
        "FACEBOOK_PAGE_ID": "your_page_id",
        "INSTAGRAM_ACCESS_TOKEN": "your_token",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id",
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
    "facebook": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/facebook.py"],
      "env": {
        "FACEBOOK_ACCESS_TOKEN": "your_token",
        "FACEBOOK_PAGE_ID": "your_page_id",
        "DRY_RUN": "false"
      }
    }
  }
}
```

---

## Testing

```bash
# Test Instagram temporary server upload (BEST - NO third-party!)
/social-poster --platform instagram "Test post" --media-path "E:/hackathon-0/Gold/ramadan_mubarak.jpg"

# Test Instagram local upload (Facebook→Instagram alternative)
/social-poster --platform instagram "Test post" --media-path "E:/hackathon-0/Gold/ramadan_mubarak.jpg"

# Test Facebook local upload
/social-poster --platform facebook "Test post" --media-path "E:/hackathon-0/Gold/ramadan_mubarak.jpg"

# Test Instagram URL upload
/social-poster --platform instagram "Test post" --media-url "https://example.com/image.jpg"
```

---

**For full documentation, see:** `.claude/skills/social-poster/SKILL.md`
