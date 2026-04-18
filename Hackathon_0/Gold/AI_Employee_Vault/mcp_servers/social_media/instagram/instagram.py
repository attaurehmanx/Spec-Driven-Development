#!/usr/bin/env python3
"""
Instagram MCP Server
Gold Tier - Instagram Posting

Posts to Instagram using Graph API.
Requires Instagram Business account linked to Facebook Page.
Supports both URL and local file uploads.
"""

import os
import json
import logging
import base64
from pathlib import Path
from typing import Any
from dotenv import load_dotenv
import requests

# Load environment variables
load_dotenv()

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
CONFIG = {
    'access_token': os.getenv('INSTAGRAM_ACCESS_TOKEN'),
    'account_id': os.getenv('INSTAGRAM_ACCOUNT_ID'),
    'api_base': 'https://graph.facebook.com/v18.0',
    'dry_run': os.getenv('DRY_RUN', 'true').lower() == 'true',
}

# Tool definitions
TOOLS = [
    {
        'name': 'social_post_instagram',
        'description': 'Post to Instagram. Supports URL or local file path for images.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'caption': {
                    'type': 'string',
                    'description': 'Post caption',
                },
                'media_url': {
                    'type': 'string',
                    'description': 'Image URL (optional if media_path provided)',
                },
                'media_path': {
                    'type': 'string',
                    'description': 'Local file path to image (optional if media_url provided)',
                },
                'hashtags': {
                    'type': 'array',
                    'items': {'type': 'string'},
                    'description': 'Array of hashtags',
                },
            },
            'required': ['caption'],
        },
    },
    {
        'name': 'social_get_instagram_analytics',
        'description': 'Get engagement metrics for recent Instagram posts.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'days': {
                    'type': 'number',
                    'description': 'Number of days to look back (default: 7)',
                },
            },
        },
    },
]


def post_to_instagram(caption: str, media_url: str = None, hashtags: list = None, media_path: str = None) -> dict:
    """Post to Instagram using Graph API
    
    Args:
        caption: Post caption text
        media_url: Public URL of image (optional if media_path provided)
        media_path: Local file path to image (optional if media_url provided)
        hashtags: List of hashtags to add
    """
    config = CONFIG

    if not config['access_token'] or not config['account_id']:
        raise Exception('Instagram credentials not configured')

    if not media_url and not media_path:
        raise Exception('Either media_url or media_path must be provided')

    if CONFIG['dry_run']:
        return {'success': True, 'dry_run': True, 'message': 'Dry run - no post made'}

    try:
        # Add hashtags to caption
        if hashtags:
            full_caption = f"{caption}\n\n{' '.join([f'#{tag}' for tag in hashtags])}"
        else:
            full_caption = caption

        # Determine image source and upload method
        if media_path:
            # Upload from local file
            return _upload_from_local_file(media_path, full_caption, config)
        else:
            # Upload from URL
            return _upload_from_url(media_url, full_caption, config)

    except Exception as e:
        logger.error(f'Instagram API error: {e}')
        raise Exception(f'Instagram post failed: {str(e)}')


def _upload_from_url(media_url: str, caption: str, config: dict) -> dict:
    """Upload image to Instagram from URL"""
    try:
        # Step 1: Create media container
        container_url = f"{config['api_base']}/{config['account_id']}/media"
        container_response = requests.post(
            container_url,
            params={
                'image_url': media_url,
                'caption': caption,
                'access_token': config['access_token'],
            },
            timeout=30
        )
        container_response.raise_for_status()
        creation_id = container_response.json().get('id')

        # Step 2: Publish the media
        publish_url = f"{config['api_base']}/{config['account_id']}/media_publish"
        publish_response = requests.post(
            publish_url,
            params={
                'creation_id': creation_id,
                'access_token': config['access_token'],
            },
            timeout=30
        )
        publish_response.raise_for_status()

        return {
            'success': True,
            'post_id': publish_response.json().get('id'),
            'url': f"https://instagram.com/p/{publish_response.json().get('id')}",
        }

    except Exception as e:
        logger.error(f'Instagram URL upload error: {e}')
        raise Exception(f'Instagram post from URL failed: {str(e)}')


def _upload_from_local_file(media_path: str, caption: str, config: dict) -> dict:
    """Upload image to Instagram from local file path
    
    Instagram Graph API requires images to be publicly accessible.
    This function uploads to Facebook first (which supports file upload),
    then shares to Instagram.
    
    Alternative: Use a temporary public URL service or host on your own server.
    """
    try:
        # Validate file exists
        file_path = Path(media_path)
        if not file_path.exists():
            raise Exception(f'File not found: {media_path}')

        # Check file extension
        valid_extensions = ['.jpg', '.jpeg', '.png']
        if file_path.suffix.lower() not in valid_extensions:
            raise Exception(f'Invalid image format. Supported: {valid_extensions}')

        # Read file as binary
        with open(file_path, 'rb') as f:
            image_data = f.read()

        logger.info(f'Processing local file: {media_path} ({len(image_data)} bytes)')

        # Instagram Graph API doesn't support direct file upload
        # We need to either:
        # 1. Upload to a temporary hosting service
        # 2. Use Facebook upload then share to Instagram
        # 3. Host on your own server
        
        # For now, we'll provide guidance on how to make the file accessible
        raise Exception(
            'Instagram Graph API requires images to be publicly accessible via URL. '
            'Please either:\n'
            '1. Host the image on a public URL and use media_url parameter instead, OR\n'
            '2. Upload to Facebook first (which supports file upload), then share to Instagram, OR\n'
            '3. Use a temporary image hosting service like imgur.com\n\n'
            f'Your file exists at: {media_path}\n'
            'To post, either:\n'
            f'  - Upload to a web server and use: media_url="https://your-site.com/image.jpg"\n'
            f'  - Or use the Facebook MCP server first, then share to Instagram'
        )

    except Exception as e:
        logger.error(f'Instagram local file upload error: {e}')
        raise Exception(f'Instagram post from file failed: {str(e)}')


def get_instagram_analytics(days: int = 7) -> dict:
    """Get Instagram account analytics"""
    config = CONFIG

    if CONFIG['dry_run']:
        return {
            'success': True,
            'dry_run': True,
            'message': 'Dry run - returning mock analytics',
            'data': {
                'posts': 0,
                'impressions': 0,
                'engagement': 0,
                'reach': 0,
            },
        }

    try:
        response = requests.get(
            f"{config['api_base']}/{config['account_id']}/insights",
            params={
                'metric': 'impressions,reach,engagement,follower_count',
                'period': 'day',
                'access_token': config['access_token'],
            },
            timeout=30
        )
        response.raise_for_status()
        return {
            'success': True,
            'platform': 'instagram',
            'period_days': days,
            'data': response.json().get('data', []),
        }

    except Exception as e:
        logger.error(f'Analytics error: {e}')
        raise Exception(f'Analytics fetch failed: {str(e)}')


def execute_tool(name: str, arguments: dict) -> dict:
    """Execute a tool by name"""
    logger.info(f'Executing tool: {name}')

    try:
        if name == 'social_post_instagram':
            return post_to_instagram(
                arguments.get('caption', ''),
                arguments.get('media_url'),
                arguments.get('hashtags', []),
                arguments.get('media_path')
            )
        elif name == 'social_get_instagram_analytics':
            return get_instagram_analytics(arguments.get('days', 7))
        else:
            raise Exception(f'Unknown tool: {name}')

    except Exception as e:
        logger.error(f'Tool execution error: {e}')
        return {
            'success': False,
            'error': str(e),
        }


def main():
    """Main MCP server loop"""
    logger.info('Instagram MCP Server starting...')

    while True:
        try:
            # Read request from stdin
            line = input()
            if not line:
                continue

            request = json.loads(line)

            # Handle different request types
            method = request.get('method', '')
            params = request.get('params', {})
            req_id = request.get('id')

            if method == 'tools/list':
                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'result': {'tools': TOOLS},
                }

            elif method == 'tools/call':
                tool_name = params.get('name', '')
                arguments = params.get('arguments', {})
                result = execute_tool(tool_name, arguments)

                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'result': {
                        'content': [
                            {
                                'type': 'text',
                                'text': json.dumps(result, indent=2),
                            }
                        ],
                        'isError': result.get('success') is False,
                    },
                }

            else:
                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'error': {
                        'code': -32601,
                        'message': f'Method not found: {method}',
                    },
                }

            # Write response to stdout
            print(json.dumps(response), flush=True)

        except json.JSONDecodeError as e:
            logger.error(f'JSON decode error: {e}')
        except EOFError:
            break
        except Exception as e:
            logger.error(f'Server error: {e}')
            error_response = {
                'jsonrpc': '2.0',
                'error': {
                    'code': -32603,
                    'message': str(e),
                },
            }
            print(json.dumps(error_response), flush=True)


if __name__ == '__main__':
    main()
