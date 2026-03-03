"""
MCP Server Instance and Lifecycle Management

This module creates and configures the FastMCP server instance that exposes
task management tools to AI agents.
"""

import logging
from mcp.server.fastmcp import FastMCP

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

# Create FastMCP server instance
mcp = FastMCP("task-tools-server")

logger.info("MCP Task Tools Server initialized")
