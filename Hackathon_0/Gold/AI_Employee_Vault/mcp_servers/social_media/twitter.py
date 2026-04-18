#!/usr/bin/env python3
"""
Twitter/X MCP Server
Gold Tier - Twitter Posting

Posts to Twitter/X using API v2.
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
    'api_key': os.getenv('TWITTER_API_KEY'),
    'api_secret': os.getenv('TWITTER_API_SECRET'),
    'access_token': os.getenv('TWITTER_ACCESS_TOKEN'),
    'access_token_secret': os.getenv('TWITTER_ACCESS_TOKEN_SECRET'),
    'bearer_token': os.getenv('TWITTER_BEARER_TOKEN'),
    'api_base': 'https://api.twitter.com/2',
    'dry_run': os.getenv('DRY_RUN', 'true').lower() == 'true',
}

# Tool definitions
TOOLS = [
    {
        'name': 'social_post_twitter',
        'description': 'Post to Twitter/X. Max 280 characters for text.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'text': {
                    'type': 'string',
                    'description': 'Tweet content (max 280 characters)',
                },
                'media_url': {
                    'type': 'string',
                    'description': 'Image or video URL (optional)',
                },
                'reply_to': {
                    'type': 'string',
                    'description': 'Tweet ID to reply to (optional)',
                },
            },
            'required': ['text'],
        },
    },
    {
        'name': 'social_get_twitter_analytics',
        'description': 'Get engagement metrics for recent Twitter posts.',
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


def post_to_twitter(text: str, media_url: str = None, reply_to: str = None) -> dict:
    """Post to Twitter/X using API v2"""
    config = CONFIG

    if not config['bearer_token']:
        raise Exception('Twitter credentials not configured')

    if CONFIG['dry_run']:
        return {'success': True, 'dry_run': True, 'message': 'Dry run - no post made'}

    try:
        # Truncate if too long
        tweet_text = text[:277] + '...' if len(text) > 280 else text

        payload = {'text': tweet_text}

        if reply_to:
            payload['reply'] = {'in_reply_to_tweet_id': reply_to}

        headers = {
            'Authorization': f"Bearer {config['bearer_token']}",
            'Content-Type': 'application/json',
            'User-Agent': 'GoldTierAIEmployee',
        }

        response = requests.post(
            f"{config['api_base']}/tweets",
            json=payload,
            headers=headers,
            timeout=30
        )
        response.raise_for_status()
        tweet_id = response.json().get('data', {}).get('id')

        return {
            'success': True,
            'tweet_id': tweet_id,
            'url': f"https://twitter.com/i/web/status/{tweet_id}",
        }

    except Exception as e:
        logger.error(f'Twitter API error: {e}')
        raise Exception(f'Twitter post failed: {str(e)}')


def get_twitter_analytics(days: int = 7) -> dict:
    """Get Twitter account analytics"""
    config = CONFIG

    if CONFIG['dry_run']:
        return {
            'success': True,
            'dry_run': True,
            'message': 'Dry run - returning mock analytics',
            'data': {
                'tweets': 0,
                'impressions': 0,
                'engagements': 0,
                'likes': 0,
                'retweets': 0,
            },
        }

    try:
        # Note: Twitter API v2 analytics requires elevated access
        # This is a simplified version
        return {
            'success': True,
            'platform': 'twitter',
            'period_days': days,
            'message': 'Twitter analytics requires elevated API access',
            'data': {},
        }

    except Exception as e:
        logger.error(f'Analytics error: {e}')
        raise Exception(f'Analytics fetch failed: {str(e)}')


def execute_tool(name: str, arguments: dict) -> dict:
    """Execute a tool by name"""
    logger.info(f'Executing tool: {name}')

    try:
        if name == 'social_post_twitter':
            return post_to_twitter(
                arguments.get('text', ''),
                arguments.get('media_url'),
                arguments.get('reply_to')
            )
        elif name == 'social_get_twitter_analytics':
            return get_twitter_analytics(arguments.get('days', 7))
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
    logger.info('Twitter MCP Server starting...')

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
