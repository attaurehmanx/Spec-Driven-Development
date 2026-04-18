# Instagram Local File Upload - MCP Solutions

## Quick Answer

**Yes, you CAN upload local images to Instagram via MCP!** Here are your options:

---

## **Solution 1: `instagram-local` MCP Server (RECOMMENDED)**

### **What It Is**
A dedicated MCP server that accepts local file paths and handles the Facebook→Instagram workflow automatically.

### **Available Tools**
| Tool | Description |
|------|-------------|
| `instagram_post_local` | Post to Instagram from local file (uploads to Facebook first, then Instagram) |
| `instagram_post_url` | Post to Instagram from public URL |
| `facebook_post_local` | Post to Facebook from local file |

### **How to Use**

#### **Via Qwen Code Skill**
```bash
/social-poster --platform instagram "Ramadan Mubarak!" --media-path "C:/Users/FAIZ REHMAN/Pictures/ramadan.jpg"
```

#### **Via MCP Tool Call**
```
Use instagram_post_local with:
- caption: "Ramadan Mubarak! 🌙✨"
- media_path: "C:/Users/FAIZ REHMAN/Pictures/ramadan.jpg"
- hashtags: ["RamadanMubarak", "BlessedMonth"]
```

### **Setup**
Add to your `%APPDATA%\qwen\mcp.json`:

```json
{
  "mcpServers": {
    "instagram-local": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram_local.py"],
      "env": {
        "FACEBOOK_ACCESS_TOKEN": "your_facebook_token",
        "FACEBOOK_PAGE_ID": "your_page_id",
        "INSTAGRAM_ACCESS_TOKEN": "your_instagram_token",
        "INSTAGRAM_ACCOUNT_ID": "your_account_id",
        "DRY_RUN": "false"
      }
    }
  }
}
```

### **How It Works**
```
User → MCP Tool Call → Upload to Facebook → Get URL → Post to Instagram → Done!
```

---

## **Solution 2: Standalone Helper Script**

### **What It Is**
A Python script that posts local images to Instagram without MCP.

### **Usage**
```bash
cd AI_Employee_Vault/mcp_servers/social_media
python post_instagram_from_local.py "C:/path/to/image.jpg" "Your caption here"
```

### **Example**
```bash
python post_instagram_from_local.py "E:/hackathon-0/Gold/ramadan_mubarak.jpg" "Ramadan Mubarak! 🌙"
```

---

## **Solution 3: Original Instagram MCP (URL Only)**

### **What It Is**
The standard `instagram` MCP server that requires public URLs.

### **Usage**
```
Use social_post_instagram with:
- caption: "Your caption"
- media_url: "https://example.com/image.jpg"
```

### **Workaround for Local Files**
1. Upload image to hosting service (Imgur, your website, etc.)
2. Get public URL
3. Use MCP tool with `media_url`

---

## **Comparison**

| Method | Local File Support | MCP Protocol | Ease of Use |
|--------|-------------------|--------------|-------------|
| **`instagram-local` MCP** | ✅ Yes | ✅ Yes | ⭐⭐⭐⭐⭐ |
| **Helper Script** | ✅ Yes | ❌ No | ⭐⭐⭐⭐ |
| **Original Instagram MCP** | ❌ No (URL only) | ✅ Yes | ⭐⭐⭐ |

---

## **Detailed Workflow: `instagram-local` MCP**

### **Step-by-Step Process**

```
┌─────────────────────────────────────────────────────────────┐
│ 1. User calls MCP tool                                      │
│    instagram_post_local(caption, media_path, hashtags)      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. MCP server reads local file                              │
│    - Validates file exists                                   │
│    - Checks file format (.jpg, .jpeg, .png)                 │
│    - Reads binary data                                       │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. Upload to Facebook                                       │
│    POST /{page_id}/photos                                   │
│    - file: (binary image data)                              │
│    - message: "caption"                                     │
│    - Returns: post_id, photo_url                            │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. Get Facebook Photo URL                                   │
│    https://graph.facebook.com/{post_id}/picture?type=normal │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 5. Create Instagram Media Container                         │
│    POST /{account_id}/media                                 │
│    - image_url: (Facebook photo URL)                        │
│    - caption: "caption + hashtags"                          │
│    - Returns: creation_id                                   │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 6. Publish to Instagram                                     │
│    POST /{account_id}/media_publish                         │
│    - creation_id: (from step 5)                             │
│    - Returns: post_id, URL                                  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 7. Return Result to User                                    │
│    {                                                        │
│      "success": true,                                       │
│      "facebook_post_id": "...",                             │
│      "facebook_url": "...",                                 │
│      "instagram_post_id": "...",                            │
│      "instagram_url": "..."                                 │
│    }                                                        │
└─────────────────────────────────────────────────────────────┘
```

---

## **Example Usage in Qwen Code**

### **Using `/social-poster` Skill**

```bash
# Post local image to Instagram
/social-poster --platform instagram "Product launch announcement" --media-path "C:/Users/FAIZ REHMAN/Pictures/product.jpg"
```

**What happens:**
1. Skill generates caption and hashtags
2. Saves to `Pending_Approval/` folder
3. Shows preview with AskUserQuestion
4. User approves
5. Calls `instagram_post_local` MCP tool
6. Uploads to Facebook → Instagram
7. Logs result and moves to `Posted/`

### **Direct MCP Tool Call**

If you have the `instagram-local` MCP server configured:

```
I want to post an image to Instagram.

Use the instagram_post_local tool with:
- caption: "Beautiful sunset tonight! 🌅"
- media_path: "C:/Users/FAIZ REHMAN/Pictures/sunset.jpg"
- hashtags: ["sunset", "photography", "nature"]
```

---

## **Testing**

### **Test `instagram-local` MCP Server**

```bash
cd AI_Employee_Vault/mcp_servers/social_media

# Test direct function call
python -c "from instagram_local import instagram_post_local; print(instagram_post_local('Test', 'C:/path/to/image.jpg'))"
```

### **Test Helper Script**

```bash
python post_instagram_from_local.py "C:/path/to/image.jpg" "Test caption"
```

---

## **Files Overview**

| File | Purpose | Uses MCP | Local Upload |
|------|---------|----------|--------------|
| `instagram_local.py` | ✅ MCP server for local uploads | ✅ Yes | ✅ Yes |
| `post_instagram_from_local.py` | 🛠️ Standalone helper script | ❌ No | ✅ Yes |
| `instagram.py` | 📡 Original Instagram MCP | ✅ Yes | ❌ No (URL only) |

---

## **Why Facebook→Instagram Workaround?**

**Instagram Graph API Limitation:**
- ❌ Does NOT support direct file uploads
- ✅ Requires images to be at public URLs

**Facebook Graph API:**
- ✅ Supports multipart file uploads
- ✅ Returns public URLs for uploaded images

**Solution:**
1. Upload to Facebook (supports files)
2. Get Facebook photo URL
3. Use URL to create Instagram post
4. Publish to Instagram

---

## **Configuration Checklist**

### **Required Credentials**
- [ ] `FACEBOOK_ACCESS_TOKEN` - Facebook Page Access Token
- [ ] `FACEBOOK_PAGE_ID` - Your Facebook Page ID
- [ ] `INSTAGRAM_ACCESS_TOKEN` - Instagram Access Token
- [ ] `INSTAGRAM_ACCOUNT_ID` - Instagram Business Account ID

### **Account Requirements**
- [ ] Facebook Page (admin access)
- [ ] Instagram Business or Creator account
- [ ] Facebook Page linked to Instagram account

### **MCP Configuration**
- [ ] Add `instagram-local` to `mcp.json`
- [ ] Set `DRY_RUN=false` for real posts
- [ ] Restart Qwen Code

---

## **Troubleshooting**

### **"File not found"**
```bash
# Check file exists
dir "C:\Users\FAIZ REHMAN\Pictures\image.jpg"

# Use absolute paths (not relative)
# ✅ Correct: C:/Users/FAIZ REHMAN/Pictures/image.jpg
# ❌ Wrong: ./image.jpg
```

### **"Invalid image format"**
- Supported: `.jpg`, `.jpeg`, `.png`
- Convert other formats before posting

### **"Facebook credentials not configured"**
- Check `.env` has `FACEBOOK_ACCESS_TOKEN` and `FACEBOOK_PAGE_ID`
- Restart MCP server

### **"Instagram credentials not configured"**
- Check `.env` has `INSTAGRAM_ACCESS_TOKEN` and `INSTAGRAM_ACCOUNT_ID`
- Restart MCP server

---

## **Summary**

| Question | Answer |
|----------|--------|
| **Can I upload local files to Instagram via MCP?** | ✅ Yes, use `instagram-local` MCP server |
| **Does it use MCP protocol?** | ✅ Yes, `instagram-local` is a full MCP server |
| **How does it work?** | Uploads to Facebook first, then uses URL for Instagram |
| **Is there a non-MCP option?** | ✅ Yes, use `post_instagram_from_local.py` script |
| **Which should I use?** | `instagram-local` MCP for integration, helper script for one-off posts |

---

**Last Updated:** 2026-03-15  
**Version:** 3.0 (MCP Local Upload Support)
