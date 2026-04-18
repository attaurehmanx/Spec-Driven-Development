#!/usr/bin/env python3
"""
Facebook MCP Server
Gold Tier - Facebook Posting

Posts to Facebook pages using Graph API.
"""

import os
import json
import logging
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
    'access_token': os.getenv('FACEBOOK_ACCESS_TOKEN'),
    'page_id': os.getenv('FACEBOOK_PAGE_ID'),
    'api_base': 'https://graph.facebook.com/v18.0',
    'dry_run': os.getenv('DRY_RUN', 'true').lower() == 'true',
}

# Tool definitions
TOOLS = [
    {
        'name': 'social_post_facebook',
        'description': 'Post to Facebook page. Requires page access token.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'text': {
                    'type': 'string',
                    'description': 'Post content',
                },
                'media_url': {
                    'type': 'string',
                    'description': 'Image or video URL (optional)',
                },
                'link': {
                    'type': 'string',
                    'description': 'Link to share (optional)',
                },
            },
            'required': ['text'],
        },
    },
    {
        'name': 'social_reply_facebook_comment',
        'description': 'Reply to a Facebook comment. Requires comment ID.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'comment_id': {
                    'type': 'string',
                    'description': 'Facebook comment ID to reply to',
                },
                'message': {
                    'type': 'string',
                    'description': 'Reply message content',
                },
            },
            'required': ['comment_id', 'message'],
        },
    },
    {
        'name': 'social_get_facebook_analytics',
        'description': 'Get engagement metrics for recent Facebook posts.',
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


def post_to_facebook(text: str, media_url: str = None, link: str = None) -> dict:
    """Post to Facebook page using Graph API"""
    config = CONFIG

    if not config['access_token'] or not config['page_id']:
        raise Exception('Facebook credentials not configured')

    if CONFIG['dry_run']:
        return {'success': True, 'dry_run': True, 'message': 'Dry run - no post made'}

    try:
        post_url = f"{config['api_base']}/{config['page_id']}/feed"
        params = {
            'message': text,
            'access_token': config['access_token'],
        }

        if link:
            params['link'] = link

        if media_url:
            # Check if video or photo
            is_video = media_url.lower().endswith(('.mp4', '.mov', '.avi'))
            if is_video:
                # Video upload
                response = requests.post(
                    f"{config['api_base']}/{config['page_id']}/videos",
                    params={
                        'file_url': media_url,
                        'published': True,
                        'description': text,
                        'access_token': config['access_token'],
                    },
                    timeout=60
                )
            else:
                # Photo upload
                response = requests.post(
                    f"{config['api_base']}/{config['page_id']}/photos",
                    params={
                        'url': media_url,
                        'published': True,
                        'message': text,
                        'access_token': config['access_token'],
                    },
                    timeout=30
                )
        else:
            response = requests.post(post_url, params=params, timeout=30)

        response.raise_for_status()
        data = response.json()

        return {
            'success': True,
            'post_id': data.get('id'),
            'url': f"https://facebook.com/{data.get('id')}",
        }

    except Exception as e:
        logger.error(f'Facebook API error: {e}')
        raise Exception(f'Facebook post failed: {str(e)}')


def reply_to_facebook_comment(comment_id: str, message: str) -> dict:
    """Reply to a Facebook comment using Graph API"""
    config = CONFIG

    if not config['access_token']:
        raise Exception('Facebook credentials not configured')

    if CONFIG['dry_run']:
        return {'success': True, 'dry_run': True, 'message': 'Dry run - no reply posted'}

    try:
        reply_url = f"{config['api_base']}/{comment_id}/comments"
        params = {
            'message': message,
            'access_token': config['access_token'],
        }

        response = requests.post(reply_url, params=params, timeout=30)
        response.raise_for_status()
        data = response.json()

        return {
            'success': True,
            'reply_id': data.get('id'),
            'url': f"https://facebook.com/{data.get('id')}",
        }

    except Exception as e:
        logger.error(f'Facebook comment reply error: {e}')
        raise Exception(f'Facebook comment reply failed: {str(e)}')


def get_facebook_analytics(days: int = 7) -> dict:
    """Get Facebook page analytics"""
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
                'clicks': 0,
            },
        }

    try:
        response = requests.get(
            f"{config['api_base']}/{config['page_id']}/insights",
            params={
                'metric': 'page_impressions_unique,page_engaged_users,page_post_engagements',
                'period': 'day',
                'access_token': config['access_token'],
            },
            timeout=30
        )
        response.raise_for_status()
        return {
            'success': True,
            'platform': 'facebook',
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
        if name == 'social_post_facebook':
            return post_to_facebook(
                arguments.get('text', ''),
                arguments.get('media_url'),
                arguments.get('link')
            )
        elif name == 'social_reply_facebook_comment':
            return reply_to_facebook_comment(
                arguments.get('comment_id', ''),
                arguments.get('message', '')
            )
        elif name == 'social_get_facebook_analytics':
            return get_facebook_analytics(arguments.get('days', 7))
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
    logger.info('Facebook MCP Server starting...')

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
