#!/usr/bin/env python3
"""
Instagram MCP Server - Local File Upload via Temporary Server
Gold Tier - Post to Instagram WITHOUT Third-Party Hosting

Workflow:
1. Start temporary local HTTP server
2. Create ngrok tunnel (public URL)
3. Use URL to create Instagram post
4. Close tunnel and server
5. Image posted to Instagram, no permanent hosting!

Requirements:
- ngrok installed: pip install pyngrok
- ngrok auth token: https://dashboard.ngrok.com/get-started/your-authtoken
"""

import os
import json
import logging
import threading
import time
import requests
from pathlib import Path
from dotenv import load_dotenv
from http.server import HTTPServer, SimpleHTTPRequestHandler

# Load environment variables
load_dotenv()

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
CONFIG = {
    'ngrok_auth_token': os.getenv('NGROK_AUTH_TOKEN'),
    'ig_access_token': os.getenv('INSTAGRAM_ACCESS_TOKEN'),
    'ig_account_id': os.getenv('INSTAGRAM_ACCOUNT_ID'),
    'api_base': 'https://graph.facebook.com/v18.0',
    'dry_run': os.getenv('DRY_RUN', 'true').lower() == 'true',
    'server_port': 8080,
}


class CustomHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler that only serves specific files"""
    
    def __init__(self, *args, serving_file=None, **kwargs):
        self.serving_file = serving_file
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        # Only serve the specific file we want
        if self.path == '/' + Path(self.serving_file).name:
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.end_headers()
            with open(self.serving_file, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        # Suppress default logging
        pass


def create_temporary_server(media_path: str) -> tuple:
    """
    Create temporary HTTP server with ngrok tunnel
    Returns (server, ngrok_url, ngrok_instance)
    """
    try:
        from pyngrok import ngrok
        
        # Set auth token
        if CONFIG['ngrok_auth_token']:
            ngrok.set_auth_token(CONFIG['ngrok_auth_token'])
        
        # Start local server in background thread
        file_path = Path(media_path)
        server_dir = str(file_path.parent)
        server_file = file_path.name
        
        os.chdir(server_dir)
        
        server = HTTPServer(('localhost', CONFIG['server_port']), 
                           lambda *args, **kwargs: CustomHandler(*args, serving_file=media_path, **kwargs))
        
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
        
        logger.info(f'Local server started on port {CONFIG["server_port"]}')
        
        # Create ngrok tunnel
        public_url = ngrok.connect(CONFIG['server_port'])
        # Extract just the URL string from the NgrokTunnel object
        ngrok_url = str(public_url.public_url) + '/' + server_file
        
        logger.info(f'ngrok tunnel created: {ngrok_url}')
        
        # Give server a moment to be ready
        time.sleep(2)
        
        return server, ngrok_url, ngrok
        
    except ImportError:
        raise Exception('pyngrok not installed. Run: pip install pyngrok')
    except Exception as e:
        raise Exception(f'Failed to create temporary server: {str(e)}')


def cleanup_server(server, ngrok_instance):
    """Shutdown server and close ngrok tunnel"""
    try:
        if server:
            server.shutdown()
        if ngrok_instance:
            ngrok_instance.disconnect()  # Close all tunnels
        logger.info('Temporary server and tunnel closed')
    except Exception as e:
        logger.warning(f'Cleanup error: {e}')


def post_to_instagram_from_url(image_url: str, caption: str, hashtags: list = None) -> dict:
    """Post to Instagram using image URL"""
    if not CONFIG['ig_access_token'] or not CONFIG['ig_account_id']:
        raise Exception('Instagram credentials not configured')
    
    # Add hashtags to caption
    if hashtags:
        full_caption = f"{caption}\n\n{' '.join([f'#{tag}' for tag in hashtags])}"
    else:
        full_caption = caption
    
    logger.info(f'Creating Instagram media container from URL...')
    
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
    
    logger.info(f'Media container created: {creation_id}')
    
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
        'url': f"https://instagram.com/p/{publish_data.get('id')}",
    }


def instagram_post_local_temporary(caption: str, media_path: str, hashtags: list = None) -> dict:
    """
    Post to Instagram from local file using temporary server.
    
    Workflow:
    1. Start local HTTP server
    2. Create ngrok tunnel (public URL)
    3. Post to Instagram
    4. Close server and tunnel
    
    Result: Image on Instagram ONLY, no permanent hosting!
    """
    if CONFIG['dry_run']:
        return {
            'success': True,
            'dry_run': True,
            'message': 'Dry run - no post made',
            'media_path': media_path,
            'caption': caption,
        }
    
    server = None
    ngrok_instance = None
    
    try:
        # Validate file
        file_path = Path(media_path)
        if not file_path.exists():
            raise Exception(f'File not found: {media_path}')
        
        # Check file format
        if file_path.suffix.lower() not in ['.jpg', '.jpeg', '.png']:
            raise Exception(f'Unsupported image format. Use .jpg, .jpeg, or .png')
        
        # Step 1: Create temporary server + tunnel
        logger.info('Creating temporary server and ngrok tunnel...')
        server, ngrok_url, ngrok_instance = create_temporary_server(media_path)
        
        logger.info(f'Public URL: {ngrok_url}')
        
        # Step 2: Post to Instagram using ngrok URL
        logger.info('Posting to Instagram...')
        ig_result = post_to_instagram_from_url(
            ngrok_url,
            caption,
            hashtags
        )
        
        logger.info(f'Instagram post successful: {ig_result["post_id"]}')
        
        return {
            'success': True,
            'instagram_post_id': ig_result['post_id'],
            'instagram_url': ig_result['url'],
            'upload_method': 'local_file_temporary_server',
            'message': 'Posted to Instagram (temporary server closed, no permanent hosting)',
        }
        
    except Exception as e:
        logger.error(f'Error during Instagram posting: {e}')
        raise Exception(f'Instagram post from local file failed: {str(e)}')
        
    finally:
        # Step 3: Cleanup - close server and tunnel
        if server or ngrok_instance:
            logger.info('Cleaning up temporary server...')
            cleanup_server(server, ngrok_instance)


def instagram_post_url(caption: str, media_url: str, hashtags: list = None) -> dict:
    """Post to Instagram from public URL"""
    if CONFIG['dry_run']:
        return {
            'success': True,
            'dry_run': True,
            'message': 'Dry run - no post made',
            'media_url': media_url,
            'caption': caption,
        }
    
    try:
        result = post_to_instagram_from_url(media_url, caption, hashtags)
        return {
            'success': True,
            'post_id': result['post_id'],
            'url': result['url'],
            'upload_method': 'url',
        }
    except Exception as e:
        logger.error(f'Error: {e}')
        raise Exception(f'Instagram post from URL failed: {str(e)}')


# MCP Server Setup
TOOLS = [
    {
        'name': 'instagram_post_local_temporary',
        'description': 'Post to Instagram from local file using temporary server. No permanent hosting - creates temporary ngrok tunnel, posts, then closes.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'caption': {'type': 'string', 'description': 'Post caption'},
                'media_path': {'type': 'string', 'description': 'Local file path (required)'},
                'hashtags': {'type': 'array', 'items': {'type': 'string'}, 'description': 'Hashtags (optional)'},
            },
            'required': ['caption', 'media_path'],
        },
    },
    {
        'name': 'instagram_post_url',
        'description': 'Post to Instagram from public URL.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'caption': {'type': 'string', 'description': 'Post caption'},
                'media_url': {'type': 'string', 'description': 'Public URL (required)'},
                'hashtags': {'type': 'array', 'items': {'type': 'string'}, 'description': 'Hashtags (optional)'},
            },
            'required': ['caption', 'media_url'],
        },
    },
]


def execute_tool(name: str, arguments: dict) -> dict:
    """Execute a tool by name"""
    logger.info(f'Executing tool: {name}')
    
    try:
        if name == 'instagram_post_local_temporary':
            return instagram_post_local_temporary(
                arguments.get('caption', ''),
                arguments.get('media_path', ''),
                arguments.get('hashtags', [])
            )
        elif name == 'instagram_post_url':
            return instagram_post_url(
                arguments.get('caption', ''),
                arguments.get('media_url', ''),
                arguments.get('hashtags', [])
            )
        else:
            raise Exception(f'Unknown tool: {name}')
    
    except Exception as e:
        logger.error(f'Tool execution error: {e}')
        return {'success': False, 'error': str(e)}


def main():
    """Main MCP server loop"""
    logger.info('Instagram Temporary Server MCP starting...')
    
    while True:
        try:
            line = input()
            if not line:
                continue
            
            request = json.loads(line)
            method = request.get('method', '')
            params = request.get('params', {})
            req_id = request.get('id')
            
            if method == 'tools/list':
                response = {'jsonrpc': '2.0', 'id': req_id, 'result': {'tools': TOOLS}}
            elif method == 'tools/call':
                result = execute_tool(params.get('name', ''), params.get('arguments', {}))
                response = {
                    'jsonrpc': '2.0',
                    'id': req_id,
                    'result': {
                        'content': [{'type': 'text', 'text': json.dumps(result, indent=2)}],
                        'isError': result.get('success') is False,
                    },
                }
            else:
                response = {'jsonrpc': '2.0', 'id': req_id, 'error': {'code': -32601, 'message': f'Method not found: {method}'}}
            
            print(json.dumps(response), flush=True)
            
        except json.JSONDecodeError as e:
            logger.error(f'JSON decode error: {e}')
        except EOFError:
            break
        except Exception as e:
            logger.error(f'Server error: {e}')
            print(json.dumps({'jsonrpc': '2.0', 'error': {'code': -32603, 'message': str(e)}}, flush=True))


if __name__ == '__main__':
    main()
