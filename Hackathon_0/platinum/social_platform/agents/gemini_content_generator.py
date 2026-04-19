"""
social_platform/agents/gemini_content_generator.py
AI Content Generation using Google Gemini (FREE API)

Gemini is perfect for social media content:
- FREE: 60 requests/minute on free tier
- Fast: Generate posts in 2-3 seconds
- Creative: Excellent for marketing content
- Multi-modal: Can analyze images too

Get your FREE API key: https://aistudio.google.com/app/apikey
"""

import os
import json
import logging
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any, Optional

try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("Install Gemini SDK: pip install google-generativeai")
    print("Then get FREE API key: https://aistudio.google.com/app/apikey")

logger = logging.getLogger(__name__)


class GeminiContentGenerator:
    """
    Generate social media content using Google Gemini AI
    
    FREE Tier: 60 requests/minute, 1500 requests/day
    Perfect for social media content generation
    """
    
    # Platform-specific content guidelines
    PLATFORM_SPECS = {
        "facebook": {
            "length": "2-4 short paragraphs",
            "emojis": "1-3 emojis",
            "hashtags": "3-5 hashtags",
            "tone": "Engaging, conversational",
            "best_for": "Announcements, stories, engagement"
        },
        "instagram": {
            "length": "Short + visual language",
            "emojis": "3-5 emojis",
            "hashtags": "5-10 hashtags",
            "tone": "Authentic, inspiring, lifestyle",
            "best_for": "Visual content, behind-the-scenes"
        },
        "twitter": {
            "length": "Under 280 characters",
            "emojis": "1-2 emojis",
            "hashtags": "2-3 hashtags",
            "tone": "Witty, timely, concise",
            "best_for": "Quick updates, thoughts, news"
        },
        "linkedin": {
            "length": "3-5 professional paragraphs",
            "emojis": "0-2 minimal emojis",
            "hashtags": "3-5 industry hashtags",
            "tone": "Professional, thought leadership",
            "best_for": "Business updates, industry insights"
        }
    }
    
    def __init__(self, api_key: str = None, model: str = "gemini-2.5-flash"):
        """
        Initialize Gemini AI
        
        Args:
            api_key: Gemini API key (or set GEMINI_API_KEY env var)
            model: Gemini model to use (default: gemini-2.5-flash)
        """
        self.api_key = api_key or os.getenv('GEMINI_API_KEY')
        self.model_name = model or os.getenv('GEMINI_MODEL', 'gemini-2.5-flash')
        
        if not self.api_key:
            logger.error("Gemini API key not found!")
            logger.error("Get FREE key at: https://aistudio.google.com/app/apikey")
            logger.error("Then add to .env: GEMINI_API_KEY=your_key_here")
            raise ValueError(
                "GEMINI_API_KEY required. "
                "Get FREE key: https://aistudio.google.com/app/apikey"
            )
        
        if not GEMINI_AVAILABLE:
            raise ImportError(
                "google-generativeai not installed. "
                "Run: pip install google-generativeai"
            )
        
        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel(self.model_name)
        
        logger.info(f"Gemini AI initialized (Model: {self.model_name})")
        logger.info("FREE Tier: 60 requests/minute, 1500 requests/day")
    
    def generate_post(self, topic: str, platform: str = "facebook", 
                     style: str = "professional", count: int = 1) -> List[str]:
        """
        Generate social media post(s)
        
        Args:
            topic: What to post about
            platform: facebook, instagram, twitter, linkedin
            style: professional, casual, funny, inspirational, exciting
            count: Number of variations to generate (default: 1)
            
        Returns:
            List of generated post options
        """
        logger.info(f"Generating {count} {platform} posts about: {topic[:50]}...")
        
        prompt = self._create_prompt(topic, platform, style, count)
        
        try:
            response = self.model.generate_content(prompt)
            posts = self._parse_response(response.text, count)
            
            logger.info(f"Successfully generated {len(posts)} post variations")
            return posts
            
        except Exception as e:
            logger.error(f"Gemini generation failed: {e}")
            return [f"Error generating content: {str(e)}"]
    
    def _create_prompt(self, topic: str, platform: str, style: str, count: int) -> str:
        """Create optimized prompt for Gemini"""
        
        specs = self.PLATFORM_SPECS.get(platform, self.PLATFORM_SPECS["facebook"])
        
        return f"""You are a professional social media content creator for AI Employee Solutions.

Generate {count} DIFFERENT post variations about this topic:

**TOPIC:** {topic}

**PLATFORM REQUIREMENTS ({platform}):**
- Length: {specs['length']}
- Emojis: {specs['emojis']}
- Hashtags: {specs['hashtags']}
- Tone: {specs['tone']}
- Best for: {specs['best_for']}

**STYLE:** {style}

**BRAND CONTEXT:**
- Company: AI Employee Solutions
- Product: Platinum Tier - Autonomous AI workers for businesses
- Features: 24/7 operations, email management, social media, Odoo accounting
- Audience: Business owners, entrepreneurs, professionals
- Voice: Innovative, helpful, trustworthy, forward-thinking

**INSTRUCTIONS:**
1. Create {count} unique variations with different angles
2. Each should have a distinct hook/approach
3. Include relevant hashtags for each
4. Make them engaging and shareable
5. Include a call-to-action

Separate each variation with "---" on its own line.

Generate the posts now:
"""
    
    def _parse_response(self, response_text: str, count: int) -> List[str]:
        """Parse Gemini response into individual posts"""
        # Split by separator
        variations = response_text.split('---')
        
        # Clean and filter empty
        posts = [v.strip() for v in variations if v.strip()]
        
        # Return requested count
        result = posts[:count] if posts else [response_text]
        
        return result
    
    def generate_from_blog(self, blog_text: str, platform: str = "facebook", 
                          count: int = 3) -> List[str]:
        """
        Generate social posts from blog post content
        
        Args:
            blog_text: Blog post text to repurpose
            platform: Target social platform
            count: Number of variations
            
        Returns:
            List of post variations
        """
        prompt = f"""You are repurposing blog content into social media posts.

**BLOG CONTENT:**
{blog_text[:2000]}  # Limit to 2000 chars

**TASK:** Create {count} engaging {platform} posts that summarize key points.

Each post should:
- Hook readers with interesting facts/quotes from the blog
- Highlight the main value proposition
- Include call-to-action to read full blog
- Follow {platform} best practices

Separate each variation with "---".

Generate posts:
"""
        
        try:
            response = self.model.generate_content(prompt)
            return self._parse_response(response.text, count)
        except Exception as e:
            logger.error(f"Blog repurposing failed: {e}")
            return [f"Error: {str(e)}"]
    
    def generate_from_image_description(self, image_desc: str, 
                                        platform: str = "instagram") -> str:
        """
        Generate caption from image description
        
        Args:
            image_desc: Description of image
            platform: Target platform
            
        Returns:
            Generated caption
        """
        prompt = f"""Create an engaging social media caption for this image.

**IMAGE DESCRIPTION:**
{image_desc}

**PLATFORM:** {platform}
**STYLE:** Engaging, authentic, brand-aligned

Include:
- Compelling hook (first line grabs attention)
- Relevant emojis (appropriate for platform)
- Story or context about the image
- Call-to-action
- Hashtags

Generate caption:
"""
        
        response = self.model.generate_content(prompt)
        return response.text
    
    def generate_weekly_content(self, topics: Dict[str, str], 
                               platform: str = "facebook",
                               style: str = "professional") -> Dict[str, str]:
        """
        Generate week's worth of content
        
        Args:
            topics: Dictionary mapping days to topics
                   e.g., {"Monday": "Product launch", "Tuesday": "Customer success"}
            platform: Target platform
            style: Content style
            
        Returns:
            Dictionary mapping days to posts
        """
        days_order = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
        
        prompt = f"""Create a week of {platform} content for AI Employee Solutions.

**TOPICS FOR EACH DAY:**
{json.dumps(topics, indent=2)}

**STYLE:** {style}

**BRAND:** AI Employee Solutions - Platinum Tier autonomous AI workers

Generate ONE post per day (7 posts total). 
Label each post clearly with the day name.
Separate each day with "---".

Generate the week's content:
"""
        
        response = self.model.generate_content(prompt)
        
        # Parse into day -> post mapping
        content_calendar = {}
        response_text = response.text
        
        for day in days_order:
            post = self._extract_day_post(response_text, day)
            if post:
                content_calendar[day] = post
        
        return content_calendar
    
    def _extract_day_post(self, text: str, day: str) -> Optional[str]:
        """Extract post for specific day from response"""
        if day not in text:
            return None
        
        # Find the day marker
        start_idx = text.find(day)
        if start_idx == -1:
            return None
        
        # Find next separator or end
        next_separator = text.find("---", start_idx)
        if next_separator == -1:
            next_separator = len(text)
        
        # Extract and clean
        post = text[start_idx:next_separator].strip()
        
        # Remove the day label from the post itself
        if post.startswith(day):
            post = post[len(day):].strip()
            if post.startswith(":"):
                post = post[1:].strip()
        
        return post
    
    def generate_engagement_response(self, comment: str, sentiment: str = "positive") -> str:
        """
        Generate response to social media comment
        
        Args:
            comment: The comment to respond to
            sentiment: positive, negative, neutral, question
            
        Returns:
            Suggested response
        """
        prompt = f"""Generate a professional social media response.

**COMMENT TO RESPOND TO:**
{comment}

**SENTIMENT:** {sentiment}

**GUIDELINES:**
- Be helpful and professional
- Address their concern/question
- Keep it concise (1-2 sentences)
- Include their name if available
- Add appropriate emoji (1 max)

Generate response:
"""
        
        response = self.model.generate_content(prompt)
        return response.text.strip()
    
    def generate_hashtags(self, topic: str, platform: str = "facebook", 
                         count: int = 10) -> List[str]:
        """
        Generate relevant hashtags for a topic
        
        Args:
            topic: Post topic
            platform: Target platform
            count: Number of hashtags
            
        Returns:
            List of hashtags
        """
        prompt = f"""Generate {count} relevant hashtags for this topic:

**TOPIC:** {topic}

**PLATFORM:** {platform}

Requirements:
- Mix of popular (1M+ posts) and niche (10K-100K posts) hashtags
- Relevant to business/technology/AI
- No banned or spammy hashtags
- Format: #Hashtag (one per line)

Generate hashtags:
"""
        
        response = self.model.generate_content(prompt)
        
        # Extract hashtags from response
        hashtags = []
        for line in response.text.split('\n'):
            line = line.strip()
            if line.startswith('#'):
                hashtags.append(line)
        
        return hashtags[:count] if hashtags else [f"#{topic.replace(' ', '')}"]


class AIContentWorkflow:
    """
    Integrate Gemini content generation with Cloud Agent workflow
    
    This class bridges AI generation with the approval system
    """
    
    def __init__(self, vault_path: str, generator: GeminiContentGenerator = None):
        """
        Initialize AI content workflow
        
        Args:
            vault_path: Path to vault directory
            generator: GeminiContentGenerator instance (or None to create)
        """
        self.vault_path = Path(vault_path)
        
        # Ensure folders exist
        (self.vault_path / 'Social_Media').mkdir(parents=True, exist_ok=True)
        (self.vault_path / 'Social_Media' / 'Drafts').mkdir(parents=True, exist_ok=True)
        (self.vault_path / 'Social_Media' / 'AI_Generated').mkdir(parents=True, exist_ok=True)
        
        # Initialize generator
        if generator:
            self.generator = generator
        else:
            try:
                self.generator = GeminiContentGenerator()
            except ValueError as e:
                logger.error(f"Cannot initialize Gemini: {e}")
                self.generator = None
    
    def create_ai_generated_post(self, topic: str, platforms: List[str], 
                                 style: str = "professional") -> List[Path]:
        """
        Generate posts with AI and create approval files
        
        Args:
            topic: What to post about
            platforms: List of platforms to post to
            style: Content style
            
        Returns:
            List of created approval file paths
        """
        if not self.generator:
            logger.error("Gemini generator not available")
            return []
        
        approval_files = []
        
        for platform in platforms:
            logger.info(f"Generating AI content for {platform}...")
            
            # Generate content with AI (SINGLE post, not multiple options)
            posts = self.generator.generate_post(topic, platform, style, count=1)
            
            # Save generated drafts
            self._save_drafts(platform, posts, topic)
            
            # Create approval file with single AI-generated post
            approval_file = self._create_approval_file(platform, posts, topic, style)
            approval_files.append(approval_file)
        
        return approval_files
    
    def _save_drafts(self, platform: str, posts: List[str], topic: str):
        """Save AI-generated drafts for reference"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_topic = "".join(c for c in topic if c.isalnum() or c in (' ', '-', '_'))[:30]
        
        draft_file = self.vault_path / 'Social_Media' / 'AI_Generated' / f"{platform}_{safe_topic}_{timestamp}.md"
        
        content = f"""---
platform: {platform}
topic: {topic}
generated: {datetime.now().isoformat()}
generated_by: gemini_ai
model: gemini-2.0-flash
---

# AI-Generated Drafts for {platform}

**Topic:** {topic}

---

"""
        for i, post in enumerate(posts, 1):
            content += f"\n## Option {i}\n\n{post}\n\n---\n"
        
        draft_file.write_text(content, encoding='utf-8')
        logger.info(f"Saved draft: {draft_file.name}")
    
    def _create_approval_file(self, platform: str, posts: List[str], 
                             topic: str, style: str) -> Path:
        """Create approval request file with AI-generated post (SINGLE post)"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"AI_SOCIAL_{platform.upper()}_{timestamp}.md"
        filepath = self.vault_path / 'Pending_Approval' / filename
        
        # Get the single post
        post_content = posts[0] if posts else "No content generated"
        
        content = f"""---
type: ai_generated_social_post
platform: {platform}
topic: {topic}
style: {style}
created: {datetime.now().isoformat()}
created_by: gemini_ai
ai_model: google_gemini_2.5_flash
status: pending
requires_local_execution: true
---

# 🤖 AI-Generated Social Media Post

**Generated by:** Google Gemini 2.5 Flash (FREE AI)  
**Platform:** {platform}  
**Topic:** {topic}  
**Style:** {style}

---

## 📝 Post Content

{post_content}

---

## ✅ To Approve

1. Review the post above
2. Edit if needed (fix any issues, adjust tone)
3. Move this file to: `vault/Approved/`

## ✏️ To Edit

Simply modify the post content above before moving to Approved.

---

## 🚀 To Reject

Move this file to: `vault/Rejected/`

---

*Generated by AI Employee Content System*
"""
        
        filepath.write_text(content, encoding='utf-8')
        logger.info(f"Created AI approval request: {filename}")
        
        return filepath
    
    def generate_and_schedule_week(self, weekly_topics: Dict[str, str],
                                   platforms: List[str] = None,
                                   style: str = "professional") -> List[Path]:
        """
        Generate week's worth of content and create approval files
        
        Args:
            weekly_topics: Dictionary mapping days to topics
            platforms: List of platforms (default: all)
            style: Content style
            
        Returns:
            List of approval file paths
        """
        if platforms is None:
            platforms = ['facebook', 'instagram', 'twitter', 'linkedin']
        
        if not self.generator:
            logger.error("Gemini generator not available")
            return []
        
        approval_files = []
        
        for platform in platforms:
            logger.info(f"Generating weekly content for {platform}...")
            
            # Generate week's content
            weekly_content = self.generator.generate_weekly_content(
                weekly_topics, platform, style
            )
            
            # Create approval file for the week
            approval_file = self._create_weekly_approval_file(
                platform, weekly_content, weekly_topics, style
            )
            approval_files.append(approval_file)
        
        return approval_files
    
    def _create_weekly_approval_file(self, platform: str, 
                                     weekly_content: Dict[str, str],
                                     topics: Dict[str, str],
                                     style: str) -> Path:
        """Create approval file for week's content"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"WEEKLY_{platform.upper()}_{timestamp}.md"
        filepath = self.vault_path / 'Pending_Approval' / filename
        
        # Format weekly content
        content_body = ""
        for day, post in weekly_content.items():
            content_body += f"\n\n## {day}\n\n**Topic:** {topics.get(day, 'TBD')}\n\n{post}\n\n---\n"
        
        content = f"""---
type: weekly_social_content
platform: {platform}
style: {style}
created: {datetime.now().isoformat()}
created_by: gemini_ai
days_included: {len(weekly_content)}
status: pending
requires_local_execution: true
---

# 📅 Weekly Social Media Content

**Platform:** {platform}  
**Generated by:** Google Gemini 2.0 Flash  
**Style:** {style}  
**Days:** {len(weekly_content)} days

---

## Content Calendar

{content_body}

---

## 📋 Instructions

### Review the Week's Content
1. Read each day's post
2. Check topic alignment
3. Verify tone and branding

### To Approve
Move this file to: `vault/Approved/`

### To Publish
Local Agent will create individual posts from this weekly plan.

---

*Generated by AI Employee Content System*
"""
        
        filepath.write_text(content, encoding='utf-8')
        logger.info(f"Created weekly content approval: {filename}")
        
        return filepath


if __name__ == '__main__':
    # Test Gemini content generation
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    print("\n" + "="*60)
    print("🤖 GEMINI AI CONTENT GENERATOR - TEST")
    print("="*60 + "\n")
    
    # Check if API key is available
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("❌ GEMINI_API_KEY not found in environment!")
        print("\n📋 Setup Instructions:")
        print("1. Get FREE API key: https://aistudio.google.com/app/apikey")
        print("2. Add to .env file: GEMINI_API_KEY=your_key_here")
        print("3. Run this script again")
        print("\n" + "="*60 + "\n")
    else:
        try:
            # Initialize generator
            generator = GeminiContentGenerator()
            
            # Test topic
            topic = "Platinum Tier AI Employee launching with 24/7 autonomous operations, email management, social media posting, and Odoo accounting integration"
            
            print(f"📝 Generating posts about:")
            print(f"   {topic[:80]}...\n")
            
            # Test Facebook post generation
            print("📘 Generating Facebook posts...")
            fb_posts = generator.generate_post(
                topic=topic,
                platform="facebook",
                style="professional_exciting",
                count=3
            )
            
            print(f"\nGenerated {len(fb_posts)} Facebook post variations:\n")
            for i, post in enumerate(fb_posts, 1):
                print(f"\n{'='*60}")
                print(f"OPTION {i}")
                print(f"{'='*60}")
                print(post)
            
            # Test Twitter post generation
            print("\n\n🐦 Generating Twitter posts...")
            twitter_posts = generator.generate_post(
                topic=topic,
                platform="twitter",
                style="witty",
                count=3
            )
            
            print(f"\nGenerated {len(twitter_posts)} Twitter post variations:\n")
            for i, post in enumerate(twitter_posts, 1):
                print(f"\n{'='*60}")
                print(f"OPTION {i}")
                print(f"{'='*60}")
                print(post)
            
            # Test LinkedIn post generation
            print("\n\n💼 Generating LinkedIn posts...")
            linkedin_posts = generator.generate_post(
                topic=topic,
                platform="linkedin",
                style="professional",
                count=3
            )
            
            print(f"\nGenerated {len(linkedin_posts)} LinkedIn post variations:\n")
            for i, post in enumerate(linkedin_posts, 1):
                print(f"\n{'='*60}")
                print(f"OPTION {i}")
                print(f"{'='*60}")
                print(post)
            
            print("\n" + "="*60)
            print("✅ TEST COMPLETE")
            print("="*60 + "\n")
            
        except Exception as e:
            print(f"\n❌ Error: {str(e)}")
            print("\nTroubleshooting:")
            print("1. Check your API key is valid")
            print("2. Ensure internet connection")
            print("3. Install: pip install google-generativeai")
