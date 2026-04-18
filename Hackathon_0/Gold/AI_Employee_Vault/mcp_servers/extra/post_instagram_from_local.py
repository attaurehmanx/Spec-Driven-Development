#!/usr/bin/env python3
"""
Instagram Post Helper - Upload Local File
Gold Tier - Workaround for Instagram API limitation

Since Instagram Graph API requires public URLs, this script:
1. Uploads image to Facebook (which supports file upload)
2. Gets the post URL
3. Provides instructions for Instagram posting
"""

import os
import json
import requests
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
CONFIG = {
    'fb_access_token': os.getenv('FACEBOOK_ACCESS_TOKEN'),
    'fb_page_id': os.getenv('FACEBOOK_PAGE_ID'),
    'ig_access_token': os.getenv('INSTAGRAM_ACCESS_TOKEN'),
    'ig_account_id': os.getenv('INSTAGRAM_ACCOUNT_ID'),
    'api_base': 'https://graph.facebook.com/v18.0',
}


def upload_to_facebook(media_path: str, caption: str) -> dict:
    """Upload image to Facebook page (supports file upload)"""
    if not CONFIG['fb_access_token'] or not CONFIG['fb_page_id']:
        raise Exception('Facebook credentials not configured')
    
    file_path = Path(media_path)
    if not file_path.exists():
        raise Exception(f'File not found: {media_path}')
    
    # Read file
    with open(file_path, 'rb') as f:
        image_data = f.read()
    
    print(f"📷 Uploading {file_path.name} to Facebook...")
    
    # Upload to Facebook
    upload_url = f"{CONFIG['api_base']}/{CONFIG['fb_page_id']}/photos"
    files = {
        'source': (file_path.name, image_data, 'image/jpeg')
    }
    data = {
        'message': caption,
        'published': True,
        'access_token': CONFIG['fb_access_token']
    }
    
    response = requests.post(upload_url, files=files, data=data, timeout=60)
    response.raise_for_status()
    result = response.json()
    
    return {
        'success': True,
        'post_id': result.get('id'),
        'url': f"https://facebook.com/{result.get('id')}"
    }


def share_to_instagram_from_facebook(fb_post_id: str) -> dict:
    """
    Share Facebook post to Instagram
    Note: This requires the Facebook page to be linked to Instagram Business account
    """
    if not CONFIG['ig_access_token'] or not CONFIG['ig_account_id']:
        raise Exception('Instagram credentials not configured')
    
    print(f"🔄 Sharing Facebook post to Instagram...")
    
    # Note: This is a simplified version
    # In reality, you'd need to create an Instagram container from the Facebook photo
    # This requires the photo to be accessible via URL
    
    return {
        'success': False,
        'message': 'Direct Facebook-to-Instagram sharing requires additional setup. '
                   'Please use the image URL from Facebook post.'
    }


def main():
    """Main function"""
    print("=" * 60)
    print("Instagram Post Helper - Local File Upload")
    print("=" * 60)
    
    # Get image path from command line or use default
    import sys
    if len(sys.argv) > 1:
        media_path = sys.argv[1]
    else:
        media_path = "E:/hackathon-0/Gold/ramadan_mubarak.jpg"
    
    # Get caption from command line or use default
    if len(sys.argv) > 2:
        caption = sys.argv[2]
    else:
        caption = "Ramadan Mubarak! 🌙✨ Wishing everyone a blessed and peaceful month. #RamadanMubarak #BlessedMonth #Ramadan2026"
    
    print(f"\n📷 Image: {media_path}")
    print(f"📝 Caption: {caption}")
    print()
    
    # Check if file exists
    if not Path(media_path).exists():
        print(f"❌ File not found: {media_path}")
        return
    
    print("✅ File exists")
    
    # Step 1: Upload to Facebook
    try:
        fb_result = upload_to_facebook(media_path, caption)
        print(f"\n✅ Facebook upload successful!")
        print(f"   Post ID: {fb_result['post_id']}")
        print(f"   URL: {fb_result['url']}")
        
        # Get the actual image URL from Facebook
        # This URL can then be used for Instagram
        photo_url = f"https://graph.facebook.com/{fb_result['post_id']}/picture?type=normal"
        print(f"\n Facebook Photo URL: {photo_url}")
        print(f"   (This URL can be used for Instagram posting)")
        
    except Exception as e:
        print(f"\n❌ Facebook upload failed: {e}")
        return
    
    # Step 2: Post to Instagram using the Facebook image URL
    print("\n" + "=" * 60)
    print("Now posting to Instagram using Facebook image URL...")
    print("=" * 60)
    
    try:
        # Use the Facebook photo URL for Instagram
        ig_result = post_to_instagram_from_url(photo_url, caption)
        
        if ig_result.get('success'):
            print(f"\n✅ Instagram post successful!")
            print(f"   Post ID: {ig_result.get('post_id')}")
            print(f"   URL: {ig_result.get('url')}")
        else:
            print(f"\n⚠️  Instagram post result: {ig_result}")
            
    except Exception as e:
        print(f"\n❌ Instagram posting failed: {e}")
        print("\n💡 Alternative: Manually post to Instagram using:")
        print(f"   1. Download image from: {fb_result['url']}")
        print(f"   2. Open Instagram app")
        print(f"   3. Create new post with the image")
        print(f"   4. Use caption: {caption}")


def post_to_instagram_from_url(image_url: str, caption: str) -> dict:
    """Post to Instagram using image URL"""
    hashtags = ['RamadanMubarak', 'BlessedMonth', 'Ramadan2026']
    full_caption = f"{caption}\n\n{' '.join([f'#{tag}' for tag in hashtags])}"
    
    # Step 1: Create media container
    container_url = f"{CONFIG['api_base']}/{CONFIG['ig_account_id']}/media"
    container_response = requests.post(
        container_url,
        params={
            'image_url': image_url,
            'caption': full_caption,
            'access_token': CONFIG['ig_access_token'],
        },
        timeout=30
    )
    container_response.raise_for_status()
    creation_id = container_response.json().get('id')
    
    print(f"   Media container created: {creation_id}")
    
    # Step 2: Publish
    publish_url = f"{CONFIG['api_base']}/{CONFIG['ig_account_id']}/media_publish"
    publish_response = requests.post(
        publish_url,
        params={
            'creation_id': creation_id,
            'access_token': CONFIG['ig_access_token'],
        },
        timeout=30
    )
    publish_response.raise_for_status()
    publish_data = publish_response.json()
    
    return {
        'success': True,
        'post_id': publish_data.get('id'),
        'url': f"https://instagram.com/p/{publish_data.get('id')}"
    }


if __name__ == '__main__':
    main()
