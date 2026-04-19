"""
batch_content_generator.py
Generate a week's worth of social media content in one run

This script uses Gemini AI to generate an entire week of social media posts
across all platforms in a single batch operation.

Features:
- Generate 7 days of content for Facebook, Instagram, Twitter, LinkedIn
- Customizable topics per day
- Platform-optimized content
- Auto-creates approval files in vault

Usage:
    python batch_content_generator.py

Prerequisites:
1. Get FREE Gemini API key: https://aistudio.google.com/app/apikey
2. Install: pip install google-generativeai
3. Add to .env: GEMINI_API_KEY=your_key_here
"""

import os
import sys
from pathlib import Path
from datetime import datetime, timedelta

# Add parent directory to path (project root)
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from social_platform.agents.gemini_content_generator import (
    GeminiContentGenerator,
    AIContentWorkflow
)


# Default weekly content themes (customize for your business)
DEFAULT_WEEKLY_THEMES = {
    "Monday": "Motivation and goal setting for business growth",
    "Tuesday": "Tips and tutorials about AI automation",
    "Wednesday": "Behind-the-scenes look at our product development",
    "Thursday": "Customer success stories and testimonials",
    "Friday": "Industry news and trends commentary",
    "Saturday": "Lighter content: team culture, work-life balance",
    "Sunday": "Reflection and preparation for the week ahead"
}

# Alternative themes for product launch
LAUNCH_WEEK_THEMES = {
    "Monday": "Teaser: Something big is coming",
    "Tuesday": "Problem statement: The challenge businesses face",
    "Wednesday": "Solution reveal: Introducing our product",
    "Thursday": "Feature spotlight: Key capability #1",
    "Friday": "Feature spotlight: Key capability #2",
    "Saturday": "Early bird offer announcement",
    "Sunday": "Last chance: Launch week wrap-up"
}


class BatchContentGenerator:
    """Generate batch social media content for the week"""
    
    def __init__(self, vault_path: str = None):
        """
        Initialize batch content generator
        
        Args:
            vault_path: Path to vault directory (default: Platinum vault)
        """
        if vault_path:
            self.vault_path = Path(vault_path)
        else:
            self.vault_path = Path(r"E:\hackathon-0\platinum\vault")
        
        # Ensure vault exists
        self.vault_path.mkdir(parents=True, exist_ok=True)
        
        # Initialize Gemini
        try:
            self.generator = GeminiContentGenerator()
            self.workflow = AIContentWorkflow(str(self.vault_path), self.generator)
            print("✓ Gemini AI initialized")
        except ValueError as e:
            print(f"❌ Gemini initialization failed: {e}")
            self.generator = None
            self.workflow = None
    
    def generate_weekly_content(self, 
                               themes: dict = None,
                               platforms: list = None,
                               style: str = "professional",
                               include_hashtags: bool = True):
        """
        Generate a full week of social media content
        
        Args:
            themes: Dictionary mapping days to topics (default: DEFAULT_WEEKLY_THEMES)
            platforms: List of platforms (default: all major platforms)
            style: Content style (professional, casual, funny, etc.)
            include_hashtags: Whether to include hashtag generation
            
        Returns:
            Dictionary with generated content by day and platform
        """
        if not self.generator:
            print("❌ Generator not initialized. Check API key.")
            return {}
        
        themes = themes or DEFAULT_WEEKLY_THEMES
        platforms = platforms or ["facebook", "instagram", "twitter", "linkedin"]
        
        print_separator("WEEKLY CONTENT GENERATION")
        
        print(f"\n📅 Generating content for {len(themes)} days")
        print(f"📱 Platforms: {', '.join(platforms)}")
        print(f"🎨 Style: {style}")
        print(f"🏷️  Hashtags: {'Included' if include_hashtags else 'Excluded'}")
        
        weekly_content = {}
        
        for day, theme in themes.items():
            print(f"\n{'='*70}")
            print(f"  📅 {day}: {theme}")
            print(f"{'='*70}")
            
            daily_posts = {}
            
            for platform in platforms:
                print(f"\n   Generating for {platform}...")
                
                try:
                    # Generate 2 variations per platform
                    posts = self.generator.generate_post(
                        topic=theme,
                        platform=platform,
                        style=style,
                        count=2
                    )
                    
                    daily_posts[platform] = posts
                    print(f"   ✓ Generated {len(posts)} {platform} posts")
                    
                except Exception as e:
                    print(f"   ❌ Error generating {platform} content: {e}")
                    daily_posts[platform] = []
            
            weekly_content[day] = daily_posts
        
        return weekly_content
    
    def save_weekly_content(self, weekly_content: dict, style: str = "professional"):
        """
        Save weekly content to vault and create approval files
        
        Args:
            weekly_content: Generated content dictionary
            style: Content style
            
        Returns:
            List of created approval file paths
        """
        if not self.workflow:
            print("❌ Workflow not initialized")
            return []
        
        print_separator("SAVING CONTENT TO VAULT")
        
        approval_files = []
        
        # Group content by platform for approval files
        platforms_content = {}
        
        for day, daily_content in weekly_content.items():
            for platform, posts in daily_content.items():
                if platform not in platforms_content:
                    platforms_content[platform] = {}
                platforms_content[platform][day] = posts
        
        # Create approval file for each platform
        for platform, daily_posts in platforms_content.items():
            print(f"\n📱 Creating approval file for {platform}...")
            
            # Create weekly approval file
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"WEEKLY_{platform.upper()}_{timestamp}.md"
            filepath = self.vault_path / 'Pending_Approval' / filename
            
            # Build content
            content_body = ""
            for day, posts in daily_posts.items():
                content_body += f"\n\n## {day}\n\n"
                for i, post in enumerate(posts, 1):
                    content_body += f"\n### Option {i}\n\n{post}\n\n---\n"
            
            # Create approval file
            content = f"""---
type: weekly_social_content
platform: {platform}
style: {style}
created: {datetime.now().isoformat()}
created_by: gemini_ai
ai_model: google_gemini_2.0_flash
days_included: {len(daily_posts)}
posts_per_day: 2
total_options: {len(daily_posts) * 2}
status: pending
requires_local_execution: true
---

# 📅 Weekly Social Media Content Plan

**Generated by:** Google Gemini 2.0 Flash (FREE AI)  
**Platform:** {platform.capitalize()}  
**Style:** {style}  
**Days:** {len(daily_posts)} days  
**Total Options:** {len(daily_posts) * 2} posts (2 per day)

---

## How to Use This File

1. **Review** all posts for each day
2. **Select** your preferred option for each day (Option 1 or Option 2)
3. **Edit** if needed (combine options, adjust tone, etc.)
4. **Approve** by moving to `/Approved/` folder
5. **Publish** - Local Agent will post selected content

---

## Content Calendar

{content_body}

---

## ✏️ Editing Instructions

### To Select Specific Options:
Add this to the frontmatter before approving:
```yaml
selected_options:
  Monday: 1
  Tuesday: 2
  Wednesday: 1
  Thursday: 1
  Friday: 2
  Saturday: 1
  Sunday: 1
```

### To Edit Individual Posts:
Simply modify the post text directly in this file before approving.

### To Schedule Specific Times:
Add posting times to frontmatter:
```yaml
posting_schedule:
  Monday: "09:00 AM EST"
  Tuesday: "11:00 AM EST"
  Wednesday: "02:00 PM EST"
  Thursday: "10:00 AM EST"
  Friday: "03:00 PM EST"
```

---

*Generated by AI Employee Batch Content System*
*{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*
"""
            
            filepath.write_text(content, encoding='utf-8')
            approval_files.append(filepath)
            
            print(f"   ✓ Created: {filename}")
        
        # Also save a draft copy for reference
        self._save_draft_copy(weekly_content)
        
        print(f"\n✓ Created {len(approval_files)} approval file(s)")
        print(f"📁 Location: {self.vault_path / 'Pending_Approval'}")
        
        return approval_files
    
    def _save_draft_copy(self, weekly_content: dict):
        """Save a draft copy for reference"""
        drafts_folder = self.vault_path / 'Social_Media' / 'AI_Generated'
        drafts_folder.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        draft_file = drafts_folder / f"WEEKLY_CONTENT_{timestamp}.md"
        
        content = f"""---
generated: {datetime.now().isoformat()}
generated_by: gemini_ai
type: weekly_content_draft
---

# Weekly Content Drafts

{self._format_drafts(weekly_content)}

---

*Reference copy - Use Pending_Approval files for actual posting*
"""
        
        draft_file.write_text(content, encoding='utf-8')
        print(f"📝 Draft copy saved: {draft_file.name}")
    
    def _format_drafts(self, weekly_content: dict) -> str:
        """Format weekly content for draft file"""
        output = ""
        
        for day, daily_content in weekly_content.items():
            output += f"\n\n## {day}\n\n"
            
            for platform, posts in daily_content.items():
                output += f"\n### {platform.capitalize()}\n\n"
                for i, post in enumerate(posts, 1):
                    output += f"#### Option {i}\n\n{post}\n\n"
        
        return output
    
    def generate_and_save(self, 
                         themes: dict = None,
                         platforms: list = None,
                         style: str = "professional"):
        """
        Generate weekly content and save to vault
        
        Args:
            themes: Day-to-topic mapping
            platforms: List of platforms
            style: Content style
            
        Returns:
            List of approval file paths
        """
        # Generate content
        weekly_content = self.generate_weekly_content(themes, platforms, style)
        
        if not weekly_content:
            print("❌ Content generation failed")
            return []
        
        # Save to vault
        approval_files = self.save_weekly_content(weekly_content, style)
        
        return approval_files


def print_separator(title: str = ""):
    """Print visual separator"""
    print("\n" + "=" * 70)
    if title:
        print(f"  {title}")
        print("=" * 70)
    else:
        print("=" * 70)


def main():
    """Main entry point"""
    print_separator("🤖 BATCH SOCIAL MEDIA CONTENT GENERATOR")
    print("\nGenerate a full week of social media content in one run!")
    print("\nThis will:")
    print("  ✓ Generate 7 days of content")
    print("  ✓ Create posts for Facebook, Instagram, Twitter, LinkedIn")
    print("  ✓ Save approval files to vault")
    print("  ✓ Provide 2 options per day per platform")
    
    # Check API key
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key or api_key == "your_gemini_api_key_here":
        print("\n❌ GEMINI_API_KEY not configured!")
        print("\n📋 Setup:")
        print("1. Get FREE key: https://aistudio.google.com/app/apikey")
        print("2. Edit .env file")
        print("3. Add: GEMINI_API_KEY=your_actual_key")
        return
    
    # Ask user for content type
    print("\n" + "-" * 70)
    print("Content Type:")
    print("  1. Standard weekly themes (motivation, tips, behind-the-scenes)")
    print("  2. Product launch week (teaser, reveal, features, offer)")
    print("  3. Custom themes (you'll specify)")
    
    choice = input("\nSelect content type (1/2/3): ").strip()
    
    if choice == "1":
        themes = DEFAULT_WEEKLY_THEMES
        print("\n✓ Using standard weekly themes")
    elif choice == "2":
        themes = LAUNCH_WEEK_THEMES
        print("\n✓ Using product launch themes")
    elif choice == "3":
        print("\nEnter custom themes (format: Day: Topic)")
        print("Example: Monday: New product announcement")
        print("(Press Enter twice when done)")
        
        themes = {}
        days = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
        
        for day in days:
            theme = input(f"{day}: ").strip()
            if theme:
                themes[day] = theme
            else:
                themes[day] = DEFAULT_WEEKLY_THEMES[day]
    else:
        themes = DEFAULT_WEEKLY_THEMES
        print("\n✓ Using standard weekly themes (default)")
    
    # Ask about platforms
    print("\n" + "-" * 70)
    print("Platforms (comma-separated):")
    print("  Options: facebook, instagram, twitter, linkedin")
    
    platform_input = input("\nPlatforms (default: all): ").strip()
    if platform_input:
        platforms = [p.strip() for p in platform_input.split(',')]
    else:
        platforms = ["facebook", "instagram", "twitter", "linkedin"]
    
    print(f"\n✓ Selected platforms: {', '.join(platforms)}")
    
    # Ask about style
    print("\n" + "-" * 70)
    print("Content Style:")
    print("  1. Professional")
    print("  2. Casual/Friendly")
    print("  3. Exciting/Enthusiastic")
    print("  4. Witty/Humorous")
    
    style_choice = input("\nSelect style (1/2/3/4): ").strip()
    style_map = {
        "1": "professional",
        "2": "casual",
        "3": "exciting",
        "4": "witty"
    }
    style = style_map.get(style_choice, "professional")
    print(f"\n✓ Using {style} style")
    
    # Confirm and generate
    print("\n" + "=" * 70)
    print("  GENERATION SUMMARY")
    print("=" * 70)
    print(f"  Days: {len(themes)}")
    print(f"  Platforms: {', '.join(platforms)}")
    print(f"  Style: {style}")
    print(f"  Posts per day: 2 options × {len(platforms)} platforms")
    print(f"  Total posts: {len(themes) * 2 * len(platforms)}")
    print("=" * 70)
    
    confirm = input("\nStart generation? (y/n): ").strip().lower()
    
    if confirm != 'y':
        print("\nGeneration cancelled.")
        return
    
    # Generate content
    generator = BatchContentGenerator()
    
    if not generator.generator:
        print("\n❌ Failed to initialize generator. Check API key.")
        return
    
    print("\n🚀 Starting content generation...")
    print("This may take 2-3 minutes (generating 56+ posts)")
    
    approval_files = generator.generate_and_save(
        themes=themes,
        platforms=platforms,
        style=style
    )
    
    # Summary
    print_separator("GENERATION COMPLETE")
    
    if approval_files:
        print(f"\n✓ Successfully generated weekly content!")
        print(f"\n📁 Approval files created:")
        for f in approval_files:
            print(f"  - {f.name}")
        
        print(f"\n📂 Location: {generator.vault_path / 'Pending_Approval'}")
        
        print("\n✅ Next Steps:")
        print("  1. Review generated posts in Pending_Approval folder")
        print("  2. Select preferred options for each day")
        print("  3. Edit if needed")
        print("  4. Move to Approved/ folder to publish")
        print("  5. Local Agent will post the content")
        
    else:
        print("\n❌ Content generation failed. Check errors above.")
    
    print_separator()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nGeneration interrupted by user.")
    except Exception as e:
        print(f"\n\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
