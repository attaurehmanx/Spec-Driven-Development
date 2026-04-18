#!/usr/bin/env python3
"""
Instagram Comment Reply MCP Server
Gold Tier - Instagram Comment Management

Dedicated MCP server for replying to Instagram comments.
Uses Instagram Graph API to reply to comments on posts.
"""

import os
import json
import logging
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
        'name': 'social_reply_instagram_comment',
        'description': 'Reply to an Instagram comment. Requires comment ID.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'comment_id': {
                    'type': 'string',
                    'description': 'Instagram comment ID to reply to',
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
        'name': 'social_get_instagram_comments',
        'description': 'Get comments for an Instagram media/post.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'media_id': {
                    'type': 'string',
                    'description': 'Instagram media/post ID',
                },
                'limit': {
                    'type': 'number',
                    'description': 'Number of comments to fetch (default: 20)',
                },
            },
            'required': ['media_id'],
        },
    },
]


def reply_to_instagram_comment(comment_id: str, message: str) -> dict:
    """Reply to an Instagram comment using Graph API
    
    Args:
        comment_id: The Instagram comment ID to reply to
        message: The reply message content
    
    Returns:
        dict with success status, reply_id, and URL
    """
    config = CONFIG

    if not config['access_token']:
        raise Exception('Instagram credentials not configured')

    if CONFIG['dry_run']:
        return {'success': True, 'dry_run': True, 'message': 'Dry run - no reply posted'}

    try:
        # Use /replies endpoint for replying to a comment
        reply_url = f"{config['api_base']}/{comment_id}/replies"
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
            'url': f"https://instagram.com/{data.get('id')}",
        }

    except Exception as e:
        logger.error(f'Instagram comment reply error: {e}')
        raise Exception(f'Instagram comment reply failed: {str(e)}')


def get_instagram_comments(media_id: str, limit: int = 20) -> dict:
    """Get comments for an Instagram media/post
    
    Args:
        media_id: The Instagram media/post ID
        limit: Number of comments to fetch (default: 20)
    
    Returns:
        dict with success status and list of comments
    """
    config = CONFIG

    if not config['access_token']:
        raise Exception('Instagram credentials not configured')

    if CONFIG['dry_run']:
        return {
            'success': True,
            'dry_run': True,
            'message': 'Dry run - returning mock comments',
            'data': [],
        }

    try:
        # Get comments from the media
        comments_url = f"{config['api_base']}/{media_id}/comments"
        params = {
            'access_token': config['access_token'],
            'limit': limit,
            'fields': 'id,text,username,timestamp,like_count',
        }

        response = requests.get(comments_url, params=params, timeout=30)
        response.raise_for_status()
        data = response.json()

        comments = []
        for comment in data.get('data', []):
            comments.append({
                'id': comment.get('id'),
                'text': comment.get('text', ''),
                'username': comment.get('username', 'Unknown'),
                'timestamp': comment.get('timestamp', ''),
                'like_count': comment.get('like_count', 0),
            })

        return {
            'success': True,
            'media_id': media_id,
            'comments': comments,
            'total_count': len(comments),
        }

    except Exception as e:
        logger.error(f'Error fetching Instagram comments: {e}')
        raise Exception(f'Failed to fetch comments: {str(e)}')


def execute_tool(name: str, arguments: dict) -> dict:
    """Execute a tool by name"""
    logger.info(f'Executing tool: {name}')

    try:
        if name == 'social_reply_instagram_comment':
            return reply_to_instagram_comment(
                arguments.get('comment_id', ''),
                arguments.get('message', '')
            )
        elif name == 'social_get_instagram_comments':
            return get_instagram_comments(
                arguments.get('media_id', ''),
                arguments.get('limit', 20)
            )
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
    logger.info('Instagram Comment MCP Server starting...')

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
