"""
MCP Server Entry Point

This script starts the MCP Task Tools server.
"""

import sys
import os

# Add parent directory to path to allow imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mcp_server.server import mcp
import logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    logger.info("Starting MCP Task Tools Server...")
    try:
        # Import tools to register them with the server
        from mcp_server.tools import task_tools

        # Run the server
        mcp.run()
    except Exception as e:
        logger.error(f"Failed to start MCP server: {str(e)}")
        raise
