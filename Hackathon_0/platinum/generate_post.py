"""
generate_post.py
Quick command-line AI post generator

Usage:
    python generate_post.py "Your topic here" facebook
    python generate_post.py "Your topic" instagram --style casual

Platforms: facebook, instagram, twitter, linkedin
Styles: professional, casual, exciting, witty
"""

import os
import sys
from pathlib import Path

# Add parent directory to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from social_platform.agents.gemini_content_generator import GeminiContentGenerator, AIContentWorkflow


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate AI social media posts')
    parser.add_argument('topic', type=str, help='What to post about')
    parser.add_argument('platform', type=str, 
                       choices=['facebook', 'instagram', 'twitter', 'linkedin'],
                       default='facebook',
                       help='Social media platform')
    parser.add_argument('--style', type=str,
                       choices=['professional', 'casual', 'exciting', 'witty'],
                       default='professional',
                       help='Content style')
    parser.add_argument('--save', action='store_true',
                       help='Save to vault approval folder')
    parser.add_argument('--media-path', type=str,
                       help='Local file path to image (for Instagram)')
    parser.add_argument('--media-url', type=str,
                       help='Public URL to image (for Instagram)')

    args = parser.parse_args()
    
    print("\n" + "=" * 70)
    print("  🤖 GEMINI AI - POST GENERATOR")
    print("=" * 70)
    print(f"\n📝 Topic: {args.topic}")
    print(f"📱 Platform: {args.platform}")
    print(f"🎨 Style: {args.style}")
    print("=" * 70)
    
    # Check API key
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key or api_key == "your_gemini_api_key_here":
        print("\n❌ GEMINI_API_KEY not configured!")
        print("\nGet FREE key: https://aistudio.google.com/app/apikey")
        print("Then add to .env: GEMINI_API_KEY=your_key")
        sys.exit(1)
    
    # Initialize generator
    try:
        generator = GeminiContentGenerator()
        print("\n✓ Gemini AI initialized")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)
    
    # Generate post
    print("\n🤖 Generating post... (5-10 seconds)\n")
    
    try:
        posts = generator.generate_post(
            topic=args.topic,
            platform=args.platform,
            style=args.style,
            count=1  # SINGLE post
        )
        
        if posts and len(posts) > 0:
            post = posts[0]
            
            print("\n" + "=" * 70)
            print("  ✨ GENERATED POST")
            print("=" * 70)
            print(f"\n{post}\n")
            print("=" * 70)
            
            # Save if requested
            if args.save:
                print("\n💾 Saving to vault...")

                vault_path = Path(r"E:\hackathon-0\platinum\vault")
                workflow = AIContentWorkflow(str(vault_path), generator)

                # Create approval file with media info
                approval_files = workflow.create_ai_generated_post(
                    topic=args.topic,
                    platforms=[args.platform],
                    style=args.style
                )

                if approval_files:
                    approval_file = approval_files[0]
                    
                    # Add media info to the file if provided
                    if args.media_path or args.media_url:
                        content = approval_file.read_text(encoding='utf-8')
                        
                        # Add media info to frontmatter
                        if 'requires_local_execution: true' in content:
                            if args.media_path:
                                content = content.replace(
                                    'requires_local_execution: true',
                                    f'requires_local_execution: true\nmedia_path: {args.media_path}'
                                )
                            if args.media_url:
                                content = content.replace(
                                    'requires_local_execution: true',
                                    f'requires_local_execution: true\nmedia_url: {args.media_url}'
                                )
                            
                            approval_file.write_text(content, encoding='utf-8')
                            print(f"📎 Media path: {args.media_path or args.media_url}")
                    
                    print(f"\n✓ Saved to: {approval_file.name}")
                    print(f"📁 Location: vault/Pending_Approval/")
                    print("\nNext: Move to Approved/ folder to publish")
            
            print("\n✅ Done!\n")
            
        else:
            print("\n❌ No post generated")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
