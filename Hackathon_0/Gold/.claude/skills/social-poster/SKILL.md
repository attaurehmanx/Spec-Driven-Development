# Social Media Poster Skill
# Gold Tier - Multi-Platform Posting

Post content across Facebook, Instagram, and Twitter/X with interactive human-in-the-loop approval.

**Note**: LinkedIn uses browser automation (Playwright) - see separate workflow below.

## Usage

```bash
/social-poster "Create a post about [topic]"
/social-poster --platform facebook "Tips to grow on Facebook"
/social-poster --platform instagram "Share behind-the-scenes content"
/social-poster --platform twitter "Quick industry insight"
/social-poster --platform instagram "Product launch" --media-path "C:/path/to/image.jpg"
```

## Interactive Workflow

**IMPORTANT**: This skill uses an interactive approval workflow. You MUST follow these steps in order:

### Step 1: Generate Content
1. Parse the user's request to extract:
   - Platform (facebook, instagram, twitter)
   - Topic/content request
   - Media path (optional, for Instagram/Facebook)
2. Read `AI_Employee_Vault/vault/Business_Goals.md` for content strategy
3. Check `AI_Employee_Vault/vault/social_media_log.txt` to avoid duplicate content
4. Generate platform-specific content following the guidelines below

### Step 2: Save to Pending Approval
1. Create a markdown file in `AI_Employee_Vault/vault/Pending_Approval/`
2. Filename format: `{platform}_{topic_slug}_{timestamp}.md`
3. Include metadata, content, hashtags, and expected engagement

### Step 3: Show Preview and Request Approval
**CRITICAL**: Use the AskUserQuestion tool to present the post and wait for user decision.

Display the generated post content clearly, then present these 4 options:

```
What would you like to do with this {platform} post?

Option 1: Approve and Post (Recommended)
- Post immediately to {platform}
- Move to Approved folder
- Log the activity

Option 2: Approve Only
- Save to Approved folder
- You will post manually later

Option 3: Request Changes
- Modify the post content
- Regenerate and review again

Option 4: Reject
- Discard this post
- Move to Rejected folder
```

### Step 4: Execute Based on User Choice

**If "Approve and Post":**
1. Move file from Pending_Approval to Approved
2. Post to the platform using the appropriate MCP tool:
   - **Facebook**: Use `facebook_post_local(message, media_path)` for local files
   - **Instagram (Local File)**: Use `instagram_post_local_temporary(caption, media_path, hashtags)`
     - Starts temporary server on your computer
     - Creates ngrok tunnel (temporary URL)
     - Posts to Instagram
     - Closes server - NO permanent hosting!
   - **Instagram (URL)**: Use `instagram_post_url(caption, media_url, hashtags)` or `social_post_instagram`
   - **Twitter**: Use `social_post_twitter(text, media_url)`
3. Update the file metadata with post_id and post_url
4. Move to Posted folder
5. Log to `social_media_log.txt`
6. Confirm success with post URL

**If "Approve Only":**
1. Move file from Pending_Approval to Approved
2. Inform user the post is saved for manual posting
3. Provide instructions on how to post later

**If "Request Changes":**
1. Ask user: "What changes would you like to make to the post?"
2. Regenerate the post with the requested modifications
3. Update the file in Pending_Approval
4. Go back to Step 3 (show preview and request approval again)
5. Continue loop until user approves or rejects

**If "Reject":**
1. Create Rejected folder if it doesn't exist
2. Move file from Pending_Approval to Rejected
3. Confirm rejection to user

### Step 5: Complete
Only after the user has made a decision and the action is executed should you complete the skill.

## Platform-Specific Guidelines

### Facebook (Community Building)
- **Frequency**: Daily
- **Length**: 100-200 words
- **Style**: Conversational, engaging, visual
- **Topics**: Business updates, customer stories, behind-the-scenes
- **Media**: Always include image or video

### Instagram (Visual Storytelling)
- **Frequency**: Daily
- **Style**: Visual-first, inspirational
- **Captions**: 50-150 words
- **Hashtags**: 10-15 relevant tags
- **Stories**: 3-5 per day

### Twitter/X (Real-Time Engagement)
- **Frequency**: 3-5x per day
- **Length**: 100-280 characters
- **Style**: Concise, timely, engaging
- **Topics**: Quick tips, industry news, engagement
- **Hashtags**: 1-3 per tweet

## Content Generation Guidelines

### Content Strategy
1. Read `AI_Employee_Vault/vault/Business_Goals.md` for:
   - Current business focus areas
   - Target audience
   - Key messaging themes
   - Social media goals per platform

2. Check `AI_Employee_Vault/vault/social_media_log.txt` to:
   - Avoid duplicate content
   - Maintain content variety
   - Learn from past performance

3. Generate content that:
   - Aligns with brand voice
   - Follows platform best practices
   - Includes clear call-to-action
   - Uses appropriate hashtags
   - Optimizes for engagement

### Content Templates by Platform

**Facebook Template:**
```
[Hook with emoji] [1-2 sentence attention grabber]

[2-3 paragraphs of engaging content]
- Conversational tone
- Tell a story or share value
- Build community connection

[Clear call-to-action]

[3-5 relevant hashtags]
```

**Instagram Template:**
```
[Visual-first caption with emojis]

[50-150 words of inspirational or storytelling content]

[Strong call-to-action]

[10-15 relevant hashtags]
[Note: Requires image URL]
```

**Twitter/X Template:**
```
[Concise, punchy message - max 280 characters]
[Include 1-3 hashtags]
[Optional: thread continuation]
```

## Post File Format

Save generated posts in this format:

```markdown
---
type: social_post
platform: facebook
topic: "Tips to grow on Facebook"
priority: medium
created: 2026-03-12T10:00:00Z
status: pending_approval
---

## Facebook Post Draft

### Content

[Generated post content here with proper formatting and emojis]

### Hashtags
#Hashtag1 #Hashtag2 #Hashtag3

### Media Requirements
- Type: Image/Video/Text-only
- Recommended: [Description of ideal media]
- Alt text: [Accessibility description if media provided]
- Media Path: [Local file path if using local upload, e.g., `C:/Users/Name/Pictures/image.jpg`]
- Media URL: [Public URL if using URL upload]

### Expected Engagement
- Impressions: 100-300
- Engagement rate: 5-10%
- Best posting time: [Day, Time]

### Notes
- [Any special considerations]
- [Platform-specific requirements]
```

## Posting Methods by Platform

### Facebook
Use MCP server tool:
- `facebook_post_local` - Post to Facebook from local file

**Posting to Facebook:**
```python
# MCP tool call with local file path
facebook_post_local(
    message="Amazing update! 🚀",
    media_path="C:/Users/YourName/Pictures/product.jpg"
)
```

### Instagram
Use MCP server tools based on image source:

**For Local Files (RECOMMENDED - NO Third-Party Hosting!):**
- `instagram_post_local_temporary` - Uses temporary server + ngrok tunnel, posts to Instagram ONLY

```python
# MCP tool call with local file path
instagram_post_local_temporary(
    caption="Amazing product launch! 🚀",
    media_path="C:/Users/YourName/Pictures/product.jpg",
    hashtags=["launch", "product"]
)
```

**How it works:**
1. Starts temporary HTTP server on your computer
2. Creates ngrok tunnel (temporary public URL)
3. Instagram fetches image from tunnel URL
4. Posts to Instagram
5. Closes server and tunnel
6. **Result:** Image on Instagram ONLY, no permanent hosting!

**For Public URLs:**
- `instagram_post_url` OR `social_post_instagram` - Post directly to Instagram from URL

```python
# MCP tool call with public URL
instagram_post_url(
    caption="Amazing product launch! 🚀",
    media_url="https://example.com/image.jpg",
    hashtags=["launch", "product"]
)
```

**Supported image formats:** `.jpg`, `.jpeg`, `.png`

**Important:**
- For local files, use `instagram_post_local_temporary` (temporary server, NO third-party hosting)
- Alternative: `instagram_post_local` (uploads to Facebook first, then Instagram)
- For URLs, use `instagram_post_url` or `social_post_instagram` (direct Instagram API)
- If MCP not available, inform user to post manually or set up MCP server.

### Twitter
Use MCP server tools:
- `social_post_twitter`

## Folder Structure

```
AI_Employee_Vault/vault/
├── Pending_Approval/     # Posts awaiting user decision
├── Approved/             # Approved but not yet posted
├── Posted/               # Successfully posted (archived)
└── Rejected/             # Rejected posts (for learning)
```

## Analytics and Logging

After successfully posting, log the activity:

**Log Entry Format** (`social_media_log.txt`):
```
---
Posted: 2026-03-12T10:30:00Z
Platform: Facebook
Page/Account: AI Employee
Topic: Tips to grow on Facebook
Post ID: 1069693056219393_122096860263074466
URL: https://facebook.com/1069693056219393_122096860263074466
Status: Success
Initial Engagement (24h): [Track later]
---
```

**Track Metrics:**
- Post ID and URL
- Timestamp
- Platform and account
- Content summary
- Engagement metrics (update after 24h)

## Safety and Quality Checks

Before posting, verify:
- ✅ Content aligns with Business_Goals.md
- ✅ No duplicate content (check log)
- ✅ Platform character limits respected
- ✅ Hashtags are relevant and appropriate
- ✅ Call-to-action is clear
- ✅ Tone matches platform expectations
- ✅ No sensitive or controversial content
- ✅ Proper grammar and spelling

**Rate Limits:**
- Max 5 posts per platform per day
- Wait 2+ hours between posts on same platform
- Check log before generating new content

## Example Usage Scenarios

**Scenario 1: Quick Facebook Post**
```
User: /social-poster --platform facebook "Share tips on social media automation"

Claude:
1. Generates Facebook post about automation tips
2. Saves to Pending_Approval/
3. Shows preview
4. Asks: "What would you like to do with this Facebook post?"
5. User selects "Approve and Post"
6. Posts to Facebook immediately
7. Returns post URL
```

**Scenario 2: Request Changes**
```
User: /social-poster --platform twitter "Quick industry insight on AI"

Claude:
1. Generates Twitter post
2. Shows preview
3. Asks for approval

User: Selects "Request Changes"

Claude: "What changes would you like to make?"

User: "Make it more technical and add statistics"

Claude:
1. Regenerates with technical focus and stats
2. Shows new preview
3. Asks for approval again

User: Selects "Approve and Post"

Claude: Posts to Twitter and confirms
```

**Scenario 3: Approve for Later**
```
User: /social-poster --platform instagram "Behind the scenes content"

Claude:
1. Generates Instagram post
2. Shows preview (notes: requires image)
3. Asks for approval

User: Selects "Approve Only"

Claude:
1. Moves to Approved folder
2. Confirms: "Post saved to Approved folder. You can post manually when you have the image ready."
```

**Scenario 4: Instagram Post with Local File (Temporary Server)**
```
User: /social-poster --platform instagram "Product launch announcement" --media-path "C:/Users/FAIZ REHMAN/Pictures/product.jpg"

Claude:
1. Generates Instagram post caption and hashtags
2. Saves to Pending_Approval/ with media_path in metadata
3. Shows preview with image path
4. Asks: "What would you like to do with this Instagram post?"

User: Selects "Approve and Post"

Claude:
1. Moves file to Approved folder
2. Calls MCP tool: instagram_post_local_temporary(caption, media_path, hashtags)
   - Starts temporary HTTP server on your computer (port 8080)
   - Creates ngrok tunnel: https://abc123.ngrok-free.app/product.jpg
   - Instagram fetches image from tunnel URL
   - Creates Instagram media container
   - Publishes to Instagram
   - Closes server and tunnel
   - Tunnel URL no longer works
3. Returns post ID and URL
4. Logs to social_media_log.txt
5. Moves to Posted folder
6. Confirms: "Posted! View at: https://instagram.com/p/..."
   - NO Facebook post created
   - NO permanent third-party hosting
```

## Implementation Checklist

When this skill is invoked, follow this exact sequence:

**Phase 1: Preparation**
- [ ] Parse user request (platform, topic)
- [ ] Read Business_Goals.md for strategy
- [ ] Check social_media_log.txt for duplicates
- [ ] Verify platform is supported

**Phase 2: Content Generation**
- [ ] Generate platform-specific content
- [ ] Follow platform guidelines (length, tone, hashtags)
- [ ] Include clear call-to-action
- [ ] Add expected engagement metrics
- [ ] Create filename: `{platform}_{slug}_{timestamp}.md`

**Phase 3: Save and Preview**
- [ ] Save to `AI_Employee_Vault/vault/Pending_Approval/`
- [ ] Display post content to user
- [ ] Show character count and hashtag count

**Phase 4: Interactive Approval (CRITICAL)**
- [ ] Use AskUserQuestion tool with 4 options:
  1. Approve and Post
  2. Approve Only
  3. Request Changes
  4. Reject
- [ ] WAIT for user response (do not proceed without selection)

**Phase 5: Execute Action**
- [ ] If "Approve and Post": Post immediately and log
- [ ] If "Approve Only": Move to Approved folder
- [ ] If "Request Changes": Ask what to change, regenerate, go back to Phase 3
- [ ] If "Reject": Move to Rejected folder

**Phase 6: Completion**
- [ ] Confirm action taken
- [ ] Provide post URL (if posted)
- [ ] Update log file
- [ ] Archive to appropriate folder

## Critical Requirements

**DO:**
- ✅ Always use AskUserQuestion for approval
- ✅ Wait for user decision before proceeding
- ✅ Show full post preview before asking for approval
- ✅ Support the "Request Changes" loop
- ✅ Log all posted content
- ✅ Move files to correct folders based on decision
- ✅ Use Page Access Token for Facebook (not user token)

**DON'T:**
- ❌ Never post without explicit user approval
- ❌ Never skip the approval step
- ❌ Never assume user wants to post immediately
- ❌ Never complete the skill before user makes a decision
- ❌ Never post duplicate content

## Troubleshooting

**Issue: Facebook 403 Error**
- Solution: Ensure using Page Access Token, not User Token
- Get page token from `/me/accounts` endpoint

**Issue: Character limit exceeded**
- Solution: Truncate content and add "..." or regenerate shorter version

**Issue: No Business_Goals.md found**
- Solution: Use general best practices, inform user to create goals file

**Issue: MCP server not available**
- Solution: For Facebook, use Python script. For others, save to Approved for manual posting.

## Success Criteria

A successful execution means:
1. ✅ Post generated following platform guidelines
2. ✅ User reviewed and made a decision
3. ✅ Action executed based on user choice
4. ✅ Content logged and archived properly
5. ✅ User received confirmation with post URL (if posted)

---

## LinkedIn Posting (Playwright Browser Automation)

**Important**: LinkedIn uses a different workflow than Facebook/Instagram/Twitter because LinkedIn API access requires OAuth app approval which is difficult to obtain.

### LinkedIn Workflow

**This skill does NOT handle LinkedIn posting directly.** Instead, LinkedIn uses browser automation via Playwright.

**How to post to LinkedIn:**

1. **Generate Content**: Use this skill to generate LinkedIn post content
   - `/social-poster --platform linkedin "Your topic"`
   - Content will be generated following LinkedIn best practices
   - Select "Approve Only" (not "Approve and Post")

2. **Content Saved**: Post saved to `Approved/linkedin/` folder

3. **Manual Posting via Playwright**:
   ```bash
   python AI_Employee_Vault/watchers/linkedin_poster.py ./AI_Employee_Vault/vault
   ```
   - Opens browser window
   - Logs into LinkedIn (first time only)
   - Automates clicking "Start a post"
   - Fills in your content
   - Clicks "Post"
   - Moves file to `Done/linkedin/`

### LinkedIn Content Guidelines

- **Frequency**: 3-5x per week
- **Length**: 150-300 words
- **Structure**: Hook → Value → CTA → Hashtags
- **Topics**: Business insights, thought leadership, industry trends
- **Best Times**: Tuesday-Thursday, 9-11 AM

### LinkedIn Template
```
[Professional hook - thought-provoking question or statement]

[Value-driven content - 150-300 words]
• Industry insights
• Business lessons
• Professional growth tips

[Call-to-action for engagement]

[3-5 professional hashtags]
```

### LinkedIn Watcher

The `linkedin_watcher.py` creates daily reminders to post:
```bash
python AI_Employee_Vault/watchers/linkedin_watcher.py ./AI_Employee_Vault/vault
```

This creates a task file every 24 hours reminding you to create a LinkedIn post.

### Why Playwright Instead of API?

- LinkedIn API requires OAuth app approval (difficult to get)
- Browser automation works immediately after login
- No API credentials needed
- Persistent browser session (login once)
- More reliable for individual users

### Setup Requirements

```bash
# Install Playwright
pip install playwright

# Install browser (or use system Edge on Windows)
playwright install chromium
```

For detailed LinkedIn integration documentation, see:
`AI_Employee_Vault/documentation/LinkedIn_Integration.md`

---

**Version**: 2.0 (Interactive Approval Workflow)
**Last Updated**: 2026-03-14
**Status**: Production Ready
