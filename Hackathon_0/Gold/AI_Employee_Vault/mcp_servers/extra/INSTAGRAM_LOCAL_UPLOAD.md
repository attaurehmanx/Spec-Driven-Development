# Instagram Posting - Local File Upload Guide

## Overview

The Instagram MCP server now supports **both URL and local file uploads**. You can post images directly from your computer without needing to host them on a public URL first.

---

## Features

### ✅ What's New

| Feature | Before | After |
|---------|--------|-------|
| **Image Source** | URL only | URL **or** local file path |
| **Upload Method** | `image_url` parameter | `image_url` **or** multipart form data |
| **Supported Formats** | JPG, PNG | JPG, JPEG, PNG |
| **Max Size** | API limits | API limits (typically 8MB) |

---

## Usage

### Method 1: Local File Upload (NEW!)

```bash
/social-poster --platform instagram "Product launch announcement" --media-path "C:/Users/FAIZ REHMAN/Pictures/product.jpg"
```

**Or via MCP tool call:**
```python
social_post_instagram(
    caption="Amazing product launch! 🚀",
    media_path="C:/Users/FAIZ REHMAN/Pictures/product.jpg",
    hashtags=["#launch", "#product", "#innovation"]
)
```

### Method 2: URL Upload (Original)

```bash
/social-poster --platform instagram "Check out our website" --media-url "https://example.com/image.jpg"
```

**Or via MCP tool call:**
```python
social_post_instagram(
    caption="Check out our website! 🌐",
    media_url="https://example.com/image.jpg",
    hashtags=["#website", "#online"]
)
```

---

## Supported Image Formats

| Format | Extension | Support |
|--------|-----------|---------|
| JPEG | `.jpg`, `.jpeg` | ✅ Full |
| PNG | `.png` | ✅ Full |
| GIF | `.gif` | ❌ Not supported |
| BMP | `.bmp` | ❌ Not supported |
| WEBP | `.webp` | ❌ Not supported |

**Image Requirements:**
- **Minimum size:** 320x320 pixels
- **Maximum size:** 4096x4096 pixels
- **Aspect ratio:** Between 1.91:1 and 4:5
- **File size:** Under 8MB

---

## How It Works

### Local File Upload Flow

```
1. User provides local file path
   ↓
2. MCP server reads file as binary
   ↓
3. Creates multipart form data request
   ↓
4. POST to Instagram Graph API:
   /{account_id}/media
   - file: (binary image data)
   - caption: "Your caption"
   ↓
5. Receives creation_id
   ↓
6. POST to /{account_id}/media_publish
   - creation_id: <from step 5>
   ↓
7. Returns post_id and URL
```

### URL Upload Flow

```
1. User provides public URL
   ↓
2. POST to Instagram Graph API:
   /{account_id}/media
   - image_url: "https://..."
   - caption: "Your caption"
   ↓
3. Receives creation_id
   ↓
4. POST to /{account_id}/media_publish
   ↓
5. Returns post_id and URL
```

---

## Configuration

### Environment Variables

Add to your `.env` file:

```bash
# Instagram Business Account
INSTAGRAM_ACCESS_TOKEN=EAAY...  # Facebook Page Access Token
INSTAGRAM_ACCOUNT_ID=17841446919185197  # Instagram Business Account ID

# Testing
DRY_RUN=true  # Set to false for real posts
```

### MCP Server Configuration

Add to `%APPDATA%\qwen\mcp.json`:

```json
{
  "mcpServers": {
    "instagram": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram.py"],
      "env": {
        "INSTAGRAM_ACCESS_TOKEN": "your_token_here",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id_here",
        "DRY_RUN": "false"
      }
    }
  }
}
```

---

## Testing

### Run Test Script

```bash
cd AI_Employee_Vault/mcp_servers/social_media
python test_instagram_local_upload.py
```

**Test script will:**
1. ✅ Check credentials are configured
2. ✅ Find test image (ramadan_mubarak.jpg or test_image.jpg)
3. ✅ Test local file upload
4. ✅ Test URL upload
5. ✅ Show results

### Manual Test

```bash
# Test with DRY_RUN=true (safe - no real post)
python -c "from instagram import post_to_instagram; print(post_to_instagram('Test', media_path='C:/path/to/image.jpg'))"

# Should return: {'success': True, 'dry_run': True, ...}
```

---

## Example Workflow

### Complete Instagram Post Workflow

**Step 1: Generate Content**
```bash
/social-poster --platform instagram "Behind the scenes at our office" --media-path "C:/Users/FAIZ REHMAN/Pictures/office.jpg"
```

**Step 2: Review Preview**

Claude generates:
```markdown
---
type: social_post
platform: instagram
media_path: "C:/Users/FAIZ REHMAN/Pictures/office.jpg"
---

## Instagram Post Draft

### Content
Behind the scenes at our office! 🏢✨

Our team is working hard to bring you amazing products.
Stay tuned for exciting updates!

### Hashtags
#BehindTheScenes #OfficeLife #TeamWork #Innovation

### Media
- Path: C:/Users/FAIZ REHMAN/Pictures/office.jpg
- Type: Image
```

**Step 3: Approve**
User selects: "Approve and Post"

**Step 4: Upload & Post**
```python
# MCP server call
result = social_post_instagram(
    caption="Behind the scenes at our office! 🏢✨\n\n#BehindTheScenes #OfficeLife #TeamWork #Innovation",
    media_path="C:/Users/FAIZ REHMAN/Pictures/office.jpg"
)
```

**Step 5: Confirmation**
```
✅ Posted successfully!
Post ID: 18012345678901234
URL: https://instagram.com/p/ABC123xyz
```

**Step 6: Logging**
```
---
Posted: 2026-03-15T12:00:00Z
Platform: Instagram
Account: AI Employee
Post ID: 18012345678901234
URL: https://instagram.com/p/ABC123xyz
Topic: Behind the scenes
Media Path: C:/Users/FAIZ REHMAN/Pictures/office.jpg
Status: Success
---
```

---

## Troubleshooting

### Error: "File not found"

**Problem:** File path doesn't exist or is incorrect

**Solution:**
```bash
# Check file exists
dir "C:/Users/FAIZ REHMAN/Pictures/image.jpg"

# Use absolute paths (not relative)
# ✅ Correct: C:/Users/FAIZ REHMAN/Pictures/image.jpg
# ❌ Wrong: ./image.jpg or ../Pictures/image.jpg
```

### Error: "Invalid image format"

**Problem:** Image is not JPG, JPEG, or PNG

**Solution:**
- Convert image to supported format
- Check file extension matches actual format

### Error: "Failed to create media container"

**Problem:** API request failed

**Solutions:**
1. Check `INSTAGRAM_ACCESS_TOKEN` is valid
2. Verify `INSTAGRAM_ACCOUNT_ID` is correct
3. Ensure Instagram account is **Business** or **Creator** type
4. Check image size is within limits (320x320 to 4096x4096)

### Error: "Credentials not configured"

**Problem:** Environment variables not set

**Solution:**
```bash
# Add to .env file
INSTAGRAM_ACCESS_TOKEN=EAAY...
INSTAGRAM_ACCOUNT_ID=17841446919185197

# Restart MCP server / Qwen Code
```

### Error: "Account must be Business"

**Problem:** Instagram account is personal, not Business

**Solution:**
1. Open Instagram app
2. Go to Settings → Account
3. Select "Switch to Professional Account"
4. Choose "Business" category
5. Link to Facebook Page

---

## API Reference

### `post_to_instagram()`

**Signature:**
```python
def post_to_instagram(
    caption: str,
    media_url: str = None,
    hashtags: list = None,
    media_path: str = None
) -> dict
```

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `caption` | str | ✅ Yes | Post caption text |
| `media_url` | str | ❌ No | Public image URL (use `media_path` instead if local) |
| `media_path` | str | ❌ No | Local file path (use `media_url` instead if URL) |
| `hashtags` | list | ❌ No | List of hashtags (without #) |

**Returns:**
```python
{
    'success': True,
    'post_id': '18012345678901234',
    'url': 'https://instagram.com/p/ABC123xyz',
    'upload_method': 'local_file'  # or 'url'
}
```

**Raises:**
- `Exception` - If upload fails

---

## Best Practices

### 1. Use DRY_RUN for Testing
```bash
# Always test with DRY_RUN=true first
DRY_RUN=true

# After testing, set to false for real posts
DRY_RUN=false
```

### 2. Validate Images Before Posting
```python
from pathlib import Path

# Check file exists
if not Path(media_path).exists():
    raise Exception("File not found")

# Check size
size_mb = Path(media_path).stat().st_size / 1024 / 1024
if size_mb > 8:
    raise Exception("File too large (max 8MB)")
```

### 3. Use Descriptive Captions
```python
# ✅ Good
caption = "Excited to announce our new product! 🚀 " \
          "After months of development, we're ready to change the game. " \
          "Learn more at our website. #Launch #Innovation"

# ❌ Bad
caption = "Test post"
```

### 4. Optimal Hashtag Strategy
```python
# Use 10-15 relevant hashtags
hashtags = [
    'Instagram', 'Business', 'Marketing',
    'SocialMedia', 'Growth', 'Entrepreneur',
    'DigitalMarketing', 'Content', 'Branding',
    'Success', 'Motivation', 'BusinessTips'
]
```

### 5. Log Everything
```python
# Always log posts for tracking
log_entry = f"""
---
Posted: {datetime.now().isoformat()}
Platform: Instagram
Post ID: {result['post_id']}
URL: {result['url']}
Media Path: {media_path}
Status: Success
---
"""
```

---

## Security Considerations

### 1. Never Expose Credentials
```python
# ✅ Good - Load from environment
access_token = os.getenv('INSTAGRAM_ACCESS_TOKEN')

# ❌ Bad - Hardcoded
access_token = "EAAY1234567890"
```

### 2. Validate File Paths
```python
# Prevent path traversal attacks
from pathlib import Path

def validate_path(media_path: str) -> bool:
    path = Path(media_path).resolve()
    # Ensure path is within allowed directory
    allowed_base = Path("C:/Users/FAIZ REHMAN/Pictures")
    return str(path).startswith(str(allowed_base))
```

### 3. Use Human-in-the-Loop
- Always require approval before posting
- Show preview with image path
- Log all actions

---

## Related Files

| File | Purpose |
|------|---------|
| `instagram.py` | MCP server implementation |
| `test_instagram_local_upload.py` | Test script |
| `SKILL.md` | Social poster skill documentation |
| `.env` | Environment configuration |

---

## References

- [Instagram Graph API Docs](https://developers.facebook.com/docs/instagram-api)
- [Media Upload API](https://developers.facebook.com/docs/instagram-api/reference/ig-user/media)
- [Hackathon Documentation](../../../Personal%20AI%20Employee%20Hackathon%200_%20Building%20Autonomous%20FTEs%20in%202026.md)

---

**Last Updated:** 2026-03-15  
**Version:** 2.0 (Local File Upload Support)
