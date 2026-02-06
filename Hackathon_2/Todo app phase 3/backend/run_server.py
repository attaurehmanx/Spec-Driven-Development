#!/usr/bin/env python3
"""
Production startup script for the Task Management API
"""

import uvicorn
import os
from main import app

if __name__ == "__main__":
    # Use environment variable for port or default to 8000
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    print(f"Starting Task Management API server on {host}:{port}")

    uvicorn.run(
        "main:app",
        host=host,
        port=port,
        reload=False,  # Disable reload in production
        log_level="info"
    )