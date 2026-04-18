---
name: social-poster
description: Create and post social media content across Facebook, Instagram, Twitter with interactive approval workflow. Uses MCP servers for posting (Gold Tier architecture).
---

# Social Media Poster - Multi-Platform Posting

## ⚠️ CRITICAL: Follow This Skill Strictly

**DO NOT deviate from this skill's instructions.** This skill provides exact steps for posting. Always:

1. **Call MCP tools via subprocess** - do not try to use subagents or background processes
2. **Follow the workflow exactly** - Steps 1-8 must be completed in order
3. **Do not improvise** - If a tool isn't working, check the script and environment, don't create alternative approaches

**Common Mistakes to Avoid:**
- ❌ Using `task` tool to delegate MCP calls
- ❌ Starting MCP servers as background processes
- ❌ Creating scripts outside the designated folders
- ❌ Skipping approval workflow steps

---

## MCP Server Architecture (Gold Tier)

This skill uses **MCP (Model Context Protocol) servers** as the "hands" of the AI Employee. Each platform has its own MCP server:

| Platform | MCP Server | Tools |
|----------|-----------|-------|
| Facebook | `facebook/facebook.py` | `social_post_facebook`, `social_reply_facebook_comment`, `social_get_facebook_analytics` |
| Instagram | `instagram/instagram.py` | `social_post_instagram`, `social_get_instagram_analytics` |
| Instagram Comments | `instagram/instagram_comment.py` | `social_reply_instagram_comment`, `social_get_instagram_comments` |
| Instagram Local | `instagram/instagram_temporary_server.py` | `instagram_post_local_temporary`, `instagram_post_url` |
| Twitter | `twitter.py` | `social_post_twitter`, `social_get_twitter_analytics` |

### MCP Configuration

To enable MCP servers, add to your Qwen Code MCP config:

**Location:** `%APPDATA%\qwen\mcp.json` (Windows) or `~/.config/qwen/mcp.json` (Linux/Mac)

```json
{
  "mcpServers": {
    "facebook": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/facebook/facebook.py"],
      "env": {
        "FACEBOOK_ACCESS_TOKEN": "your_token",
        "FACEBOOK_PAGE_ID": "your_page_id",
        "DRY_RUN": "false"
      }
    },
    "instagram": {
      "command": "python",
      "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram/instagram.py"],
      "env": {
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

**Instagram Documentation:** Full guide available at `AI_Employee_Vault/mcp_servers/social_media/instagram/README.md`

## Workflow

### Step 1: Parse User Request
Extract from user input:
- Platform: facebook, instagram, twitter, linkedin
- Topic/content request

### Step 2: Read Content Strategy
```bash
# Read business goals for strategy
cd AI_Employee_Vault/vault
type Business_Goals.md

# Check social media log for duplicates
type social_media_log.txt
```

### Step 3: Generate Platform-Specific Content

**Facebook Template:**
```
[Hook with emoji] [1-2 sentence attention grabber]

[2-3 paragraphs of engaging content]
- Conversational tone
- Tell a story or share value

[Clear call-to-action]

[3-5 relevant hashtags]
```

**Instagram Template:**
```
[Visual-first caption with emojis]

[50-150 words of inspirational content]

[Strong call-to-action]

[10-15 relevant hashtags]
```

**Twitter/X Template:**
```
[Concise, punchy message - max 280 characters]
[Include 1-3 hashtags]
```

**LinkedIn Template:**
```
[Professional hook - thought-provoking statement]

[Value-driven content - 150-300 words]
• Industry insights
• Business lessons

[Call-to-action for engagement]

[3-5 professional hashtags]
```

### Step 4: Save to Pending Approval
Create file: `Pending_Approval/{platform}_{topic}_{timestamp}.md`

```markdown
---
type: social_post
platform: [facebook|instagram|twitter|linkedin]
topic: "[Topic]"
priority: medium
created: [ISO timestamp]
status: pending_approval
---

## [Platform] Post Draft

### Content
[Generated post content]

### Hashtags
#Hashtag1 #Hashtag2 #Hashtag3

### Expected Engagement
- Impressions: 100-300
- Engagement rate: 5-10%

### Best Posting Time
[Day, Time]
```

### Step 5: Request User Approval
Present 4 options to user:

1. **Approve and Post** - Post immediately via MCP server
2. **Approve Only** - Save for manual posting
3. **Request Changes** - Modify content
4. **Reject** - Discard post

### Step 6: Execute Based on Decision

**If "Approve and Post":**

1. Move file to `Approved/` folder
2. Post to the platform using one of the methods below
3. MCP server posts and returns post ID/URL
4. Log the post to `social_media_log.txt`
5. Move file to `Posted/` folder

**Facebook Posting Methods:**

| Method | Tool | Use Case |
|--------|------|----------|
| Reusable Script | `post_facebook.py` | Quick posting via command line (Recommended) |
| JSON-RPC Pipe | `facebook.py` via stdin | Direct MCP server call |

**Option 1: Using the Reusable Script (Recommended)**

```bash
# Basic text post
python AI_Employee_Vault/mcp_servers/social_media/facebook/post_facebook.py "Your post content here"

# Post with media URL
python AI_Employee_Vault/mcp_servers/social_media/facebook/post_facebook.py "Your post content" --media-url "https://example.com/image.png"
```

**Example:**
```bash
python E:\hackathon-0\Gold\AI_Employee_Vault\mcp_servers\social_media\facebook\post_facebook.py "🤖 AI automation is transforming businesses! Ready to automate your workflow? #AIAutomation #FutureOfWork"
```

### ⚠️ Windows Multi-Line Post Fix

**Problem:** Windows `cmd.exe` does not support multi-line strings in command arguments. Using actual newlines will truncate your post to a single line.

**Solution:** Use `\n` escape sequences for line breaks. The `post_facebook.py` script automatically converts `\n` to actual newlines.

**Example (Multi-Line Post on Windows):**
```bash
python E:\hackathon-0\Gold\AI_Employee_Vault\mcp_servers\social_media\facebook\post_facebook.py "⚡ Stop Doing Busy Work—Let AI Handle It ⚡\n\nStill manually entering data? Chasing invoices?\n\nHere's the truth: Your time is worth more than that.\n\nAI automation is no longer about \"someday\"—it's about today.\n\n🎯 What's ONE task you wish you could automate? Drop it in the comments!\n\n#AIAutomation #SmallBusiness #ProductivityHacks"
```

**Key Points:**
- Use `\n` for newlines (the script converts them automatically)
- Use `\n\n` for paragraph breaks
- Escape quotes inside the text with `\"`
- This fix is built into `post_facebook.py` - no manual conversion needed

**Option 2: Direct JSON-RPC via Pipe**

```bash
# Create JSON request and pipe to Facebook MCP server
echo {"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"social_post_facebook","arguments":{"text":"Your post content","media_url":""}}} | python AI_Employee_Vault/mcp_servers/social_media/facebook/facebook.py
```

**Example (Windows PowerShell):**
```powershell
$json = '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"social_post_facebook","arguments":{"text":"AI automation is here! #AIAutomation","media_url":""}}}}'
echo $json | python E:\hackathon-0\Gold\AI_Employee_Vault\mcp_servers\social_media\facebook\facebook.py
```

**Instagram Posting Methods:**

| Method | Tool | Use Case |
|--------|------|----------|
| URL | `social_post_instagram` | Image already hosted online |
| Local File | `instagram_post_local_temporary` | Image on computer (uses ngrok tunnel) |

**Posting via MCP Server (Direct Call):**

Call the MCP server directly using subprocess with JSON-RPC:

```bash
# Instagram Local File (via instagram_temporary_server.py)
echo '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"instagram_post_local_temporary","arguments":{"caption":"Your caption #hashtags","media_path":"E:/path/to/image.png","hashtags":["tag1","tag2"]}}}' | python instagram_temporary_server.py

# Instagram URL (via instagram_temporary_server.py)
echo '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"instagram_post_url","arguments":{"caption":"Your caption #hashtags","media_url":"https://example.com/image.png","hashtags":["tag1","tag2"]}}}' | python instagram_temporary_server.py
```

**Example (Windows):**
```bash
cd E:/hackathon-0/Gold
echo {"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"instagram_post_local_temporary","arguments":{"caption":"🚀 Building the future! #AI #Automation","media_path":"E:/hackathon-0/Gold/business_image.png","hashtags":["AI","Automation"]}}} | python AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_temporary_server.py
```

**Example (Linux/Mac):**
```bash
cd /path/to/Gold
echo '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"instagram_post_local_temporary","arguments":{"caption":"🚀 Building the future! #AI #Automation","media_path":"/path/to/business_image.png","hashtags":["AI","Automation"]}}}' | python AI_Employee_Vault/mcp_servers/social_media/instagram/instagram_temporary_server.py
```

**Steps after MCP call:**
1. MCP server posts to the platform and returns post ID/URL
2. Log the post to `social_media_log.txt`
3. Create approval file in `Approved/` folder
4. Move file to `Posted/` folder

**If "Approve Only":**
- Move to `Approved/` folder
- Inform user for manual posting

**If "Request Changes":**
- Ask: "What changes would you like?"
- Regenerate content
- Return to Step 4

**If "Reject":**
- Move to `Rejected/` folder

### Step 7: Log Activity
Append to `social_media_log.txt`:
```
---
Posted: [ISO timestamp]
Platform: [platform]
Topic: [topic]
Post ID: [id]
URL: [url]
Status: Success
---
```

### Step 8: Move to Posted
```bash
# Move completed post to Posted folder
move AI_Employee_Vault/vault/Approved\*.md AI_Employee_Vault/vault/Posted/
```

## Platform Guidelines

| Platform | Frequency | Length | Hashtags | Best Time |
|----------|-----------|--------|----------|-----------|
| Facebook | Daily | 100-200 words | 3-5 | 9-11 AM |
| Instagram | Daily | 50-150 words | 10-15 | 6-9 PM |
| Twitter | 3-5x/day | 100-280 chars | 1-3 | 8-10 AM |
| LinkedIn | 3-5x/week | 150-300 words | 3-5 | Tue-Thu 9-11 AM |

## Safety Checks

Before posting, verify:
- ✅ Content aligns with Business_Goals.md
- ✅ No duplicate content (check log)
- ✅ Character limits respected
- ✅ Hashtags are relevant
- ✅ Call-to-action is clear
- ✅ Proper grammar and spelling

## Rate Limits
- Max 5 posts per platform per day
- Wait 2+ hours between posts on same platform

## Commands Reference

| Action | Command |
|--------|---------|
| Create post | `/social-poster "topic"` |
| Platform specific | `/social-poster --platform facebook "topic"` |
| Facebook post | `python AI_Employee_Vault/mcp_servers/social_media/facebook/post_facebook.py "content"` |
| Instagram with URL | `/social-poster --platform instagram "caption" --media-url "https://..."` |
| Instagram with local file | `/social-poster --platform instagram "caption" --media-path "E:/path/to/image.png"` |
| Batch create | `/social-poster --batch 7` |

## Instagram Comment Replies

To reply to Instagram comments (engagement management):

```python
# Via MCP tool call
social_reply_instagram_comment(
    comment_id="17891269263305259",
    message="Thank you for your comment! 🙏"
)
```

**Note:** Comment replies use a separate MCP server (`instagram-comment`) for microservices architecture.

## Folder Structure
```
AI_Employee_Vault/vault/
├── Pending_Approval/
├── Approved/
├── Posted/
└── Rejected/
```
