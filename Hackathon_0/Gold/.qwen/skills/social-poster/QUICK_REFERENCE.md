# Social Poster Skill - Quick Reference

## ⚠️ CRITICAL: Follow This Skill Strictly

**DO NOT deviate from these instructions.** This skill provides exact steps.

**Always:**
1. **Call MCP tools via subprocess** - no subagents or background processes
2. **Follow the workflow exactly** - Steps 1-8 in order
3. **Do not improvise** - Check scripts/environment if issues occur

**Common Mistakes to Avoid:**
- ❌ Using `task` tool to delegate MCP calls
- ❌ Starting MCP servers as background processes
- ❌ Creating scripts outside designated folders
- ❌ Skipping approval workflow steps

---

## Updated for Instagram Microservices Architecture

**Last Updated:** 2026-03-16
**Version:** 4.1

---

## Quick Usage

### **Instagram with Local File**
```bash
/social-poster --platform instagram "Your caption" --media-path "E:/path/to/image.png"
```

### **Instagram with URL**
```bash
/social-poster --platform instagram "Your caption" --media-url "https://example.com/image.jpg"
```

### **Facebook with Image**
```bash
/social-poster --platform facebook "Your message" --media-url "https://example.com/image.jpg"
```

### **Twitter**
```bash
/social-poster --platform twitter "Your tweet text"
```

### **Instagram Comment Reply**
```python
social_reply_instagram_comment(
    comment_id="17891269263305259",
    message="Thank you! 🙏"
)
```

---

## MCP Tools by Platform

| Platform | Tool Name | Parameters | Use Case |
|----------|-----------|------------|----------|
| **Instagram (Local)** | `instagram_post_local_temporary` | `caption`, `media_path`, `hashtags` | Upload from computer (ngrok tunnel) |
| **Instagram (URL)** | `social_post_instagram` | `caption`, `media_url`, `hashtags` | Post from web URL |
| **Instagram (URL)** | `instagram_post_url` | `caption`, `media_url`, `hashtags` | Post from web URL (alternative) |
| **Facebook** | `social_post_facebook` | `text`, `media_url` | Post to Facebook page |
| **Twitter** | `social_post_twitter` | `text`, `media_url` | Post tweet |
| **IG Comments** | `social_reply_instagram_comment` | `comment_id`, `message` | Reply to comments |

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
| Image on computer | `instagram_post_local_temporary` | ✅ BEST - Temporary ngrok server, direct to Instagram |
| Image on website | `social_post_instagram` | Direct URL posting |
| Already have URL | `instagram_post_url` | Alternative URL posting |

---

## Example: Local File Upload (ngrok Tunnel)

**User Input:**
```bash
/social-poster --platform instagram "Eid Mubarak!" --media-path "E:/hackathon-0/Gold/eid_mubarak_temp.png"
```

**What Happens:**
1. Generates caption and hashtags
2. Saves to `Pending_Approval/instagram_eid_mubarak_20260316.md`
3. Shows preview with AskUserQuestion
4. User approves
5. Calls: `instagram_post_local_temporary(caption, media_path, hashtags)`
6. MCP server:
   - Starts local HTTP server (port 8080)
   - Creates ngrok tunnel (temporary public URL)
   - Instagram fetches image from tunnel
   - Posts to Instagram
   - CLOSES server and tunnel
7. Logs result to `social_media_log.txt`
8. Moves file to `Posted/`

**Result:**
- ✅ Posted to Instagram
- ❌ NO permanent hosting
- ❌ NO third-party service (temporary only)

---

## Error Handling

| Error | Action |
|-------|--------|
| File not found | "File not found at {path}. Please check the path." |
| Invalid format | "Unsupported format. Use .jpg, .jpeg, or .png" |
| Credentials error | "API credentials not configured. Check .env file." |
| MCP not available | Save to Approved/, inform user to post manually |

---

## Logging Format

```txt
---
Posted: 2026-03-16T19:39:16Z
Platform: instagram
Topic: Eid Mubarak
Post ID: 18107209900658133
URL: https://instagram.com/p/18107209900658133
Media Path: E:/hackathon-0/Gold/eid_mubarak_temp.png
Upload Method: local_file_temporary_server
Status: Success
---
```

---

## Best Practices

1. **Always use AskUserQuestion** before posting
2. **Show full preview** including image path
3. **Log every post** to social_media_log.txt
4. **Check for duplicates** before generating
5. **Use instagram_post_local_temporary** for local files
6. **Move files** to correct folders based on user decision
7. **Include hashtags** in Instagram posts (10-15 recommended)

---

## ⚠️ Windows Multi-Line Post Fix (Facebook)

**Problem:** Windows `cmd.exe` does not support multi-line strings in command arguments. Using actual newlines will truncate your post to a single line.

**Solution:** Use `\n` escape sequences for line breaks. The `post_facebook.py` script automatically converts `\n` to actual newlines.

**Example (Multi-Line Post on Windows):**
```bash
python AI_Employee_Vault/mcp_servers/social_media/facebook/post_facebook.py "⚡ Stop Doing Busy Work ⚡\n\nStill manually entering data?\n\nHere's the truth: Your time is worth more than that.\n\n🎯 What's ONE task you wish you could automate?\n\n#AIAutomation #SmallBusiness"
```

**Key Points:**
- Use `\n` for newlines (the script converts them automatically)
- Use `\n\n` for paragraph breaks
- Escape quotes inside the text with `\"`
- This fix is built into `post_facebook.py` - no manual conversion needed

---

## Supported Platforms

| Platform | Local Files | URLs | MCP Tool |
|----------|-------------|------|----------|
| Instagram | ✅ YES (ngrok) | ✅ YES | `instagram_post_local_temporary`, `social_post_instagram` |
| Facebook | ✅ YES | ✅ YES | `social_post_facebook` |
| Twitter | ⚠️ (via URL) | ✅ YES | `social_post_twitter` |
| LinkedIn | ❌ NO | ❌ NO | Browser automation (Playwright) |

---

## Configuration Required

Add to `%APPDATA%\qwen\mcp.json`:

```json
{
  "mcpServers": {
    "instagram": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram.py"],
      "env": {
        "INSTAGRAM_ACCESS_TOKEN": "your_token",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id",
        "DRY_RUN": "false"
      }
    },
    "instagram-temporary": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_temporary_server.py"],
      "env": {
        "NGROK_AUTH_TOKEN": "your_ngrok_token",
        "INSTAGRAM_ACCESS_TOKEN": "your_token",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id",
        "DRY_RUN": "false"
      }
    },
    "instagram-comment": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_comment.py"],
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

---

## Testing

```bash
# Test Instagram local file upload
/social-poster --platform instagram "Test post" --media-path "E:/hackathon-0/Gold/eid_mubarak_temp.png"

# Test Instagram URL upload
/social-poster --platform instagram "Test post" --media-url "https://example.com/image.jpg"

# Test Facebook
/social-poster --platform facebook "Test post" --media-url "https://example.com/image.jpg"

# Test Twitter
/social-poster --platform twitter "Test tweet message"
```

---

## Instagram Servers Location

```
AI_Employee_Vault/mcp_servers/social_media/instagram/
├── instagram.py                      # Main posting server
├── instagram_comment.py              # Comment replies
├── instagram_temporary_server.py     # Local file uploads (ngrok)
├── README.md                         # Full Instagram guide
└── QUICK_REFERENCE.md                # Quick reference
```

---

**For full documentation, see:** `.qwen/skills/social-poster/SKILL.md`
