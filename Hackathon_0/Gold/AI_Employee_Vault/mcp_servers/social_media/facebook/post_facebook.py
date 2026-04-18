#!/usr/bin/env python3
"""
Facebook Poster - Reusable script to post to Facebook via MCP server
Usage: python post_facebook.py "Your post text here" [--media-url "https://..."]
"""
import sys
import os
import argparse

# Import the function from same directory
sys.path.insert(0, os.path.dirname(__file__))
from facebook import post_to_facebook

def main():
    parser = argparse.ArgumentParser(description='Post to Facebook')
    parser.add_argument('text', help='Post content')
    parser.add_argument('--media-url', '-m', default='', help='Image or video URL (optional)')
    args = parser.parse_args()

    # Convert \n escape sequences to actual newlines
    text = args.text.encode().decode('unicode_escape')

    # Post to Facebook
    result = post_to_facebook(text=text, media_url=args.media_url)
    print(result)

if __name__ == '__main__':
    main()
