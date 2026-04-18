#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Facebook Comment Reply Tool
Reply to Facebook comments using the MCP server's secure function

Usage:
    python reply_facebook_comment.py <comment_id> <message>

Example:
    python reply_facebook_comment.py "122096860263074466_887671284268298" "Thanks for your comment!"
"""
import sys
import os
import json

# Set UTF-8 encoding for output
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')

# Import the reply function
from server import reply_to_facebook_comment

def main():
    if len(sys.argv) < 3:
        print("Usage: python reply_facebook_comment.py <comment_id> <message>")
        print("\nExample:")
        print('  python reply_facebook_comment.py "122096860263074466_887671284268298" "Thanks for your comment!"')
        sys.exit(1)

    comment_id = sys.argv[1]
    message = sys.argv[2]

    print(f"Replying to Facebook comment...")
    print(f"Comment ID: {comment_id}")
    print(f"Message: {message}")
    print()

    try:
        result = reply_to_facebook_comment(comment_id, message)
        if result.get('success'):
            print("Success!")
            print(f"Reply ID: {result.get('reply_id')}")
            print(f"URL: {result.get('url')}")
        else:
            print(f"Failed: {result.get('error', 'Unknown error')}")
            sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
