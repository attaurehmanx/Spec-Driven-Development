from dotenv import load_dotenv
import os

# Load environment variables from .env file BEFORE any other imports
# This ensures GEMINI_API_KEY and other env vars are available
load_dotenv()

from fastapi import FastAPI
from api.auth import router as auth_router
from api.users import router as users_router
from api.tasks import router as tasks_router
from api.protected import router as protected_router
from api.chat import router as chat_router
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import sys
from database.migrations import create_db_and_tables
from contextlib import asynccontextmanager
from utils.logging_config import logger


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for application startup and shutdown
    """
    # Startup
    logger.info("Starting Task Management API with Authentication...")
    create_db_and_tables()
    logger.info("Database tables created/reconciled")
    yield
    # Shutdown (if needed)
    logger.info("Shutting down Task Management API...")


app = FastAPI(
    title="Task Management API with Authentication",
    description="A secure task management API with JWT-based authentication",
    version="1.0.0",
    lifespan=lifespan
)


# Add CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000", "http://localhost:3001", "http://127.0.0.1:3001"],  # Allow frontend domains
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


import os
from fastapi.responses import FileResponse
from starlette.responses import RedirectResponse
import mimetypes
from pathlib import Path

# Note: For Hugging Face Spaces, file uploads are ephemeral and will not persist
# Avatar serving is now handled by custom logic which provides fallback functionality
# Consider using external storage (AWS S3, Google Cloud Storage, etc.) for production

# Create a specific route for avatar requests to handle the fallback logic
@app.get("/api/uploads/avatars/{filename}")
async def serve_avatar(filename: str):
    """
    Serve avatar files with fallback to default avatar.
    This handles the ephemeral storage issue on Hugging Face Spaces.
    """
    # Define the path to the avatar file
    avatar_path = Path("uploads/avatars") / filename

    # Check if the requested avatar file exists
    if avatar_path.exists():
        # Serve the requested avatar file
        media_type, _ = mimetypes.guess_type(str(avatar_path))
        return FileResponse(
            path=str(avatar_path),
            media_type=media_type or "application/octet-stream"
        )

    # If the requested avatar doesn't exist, serve the default avatar
    default_avatar_paths = [
        Path("uploads/avatars/default-avatar.png"),
        Path("uploads/avatars/default-avatar.svg")
    ]

    for default_path in default_avatar_paths:
        if default_path.exists():
            media_type, _ = mimetypes.guess_type(str(default_path))
            return FileResponse(
                path=str(default_path),
                media_type=media_type or "image/svg+xml"  # SVG as default for avatar
            )

    # If no default avatar exists, return a 404
    from starlette.responses import PlainTextResponse
    return PlainTextResponse("Avatar not found", status_code=404)

uploads_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "uploads"))
try:
    # Create uploads directory if it doesn't exist
    os.makedirs(uploads_path, exist_ok=True)

    # Static file serving is now handled by the custom route above for better control over avatar fallbacks
except Exception as e:
    logger.warning(f"Could not create uploads directory: {e}")


# Include routers
app.include_router(auth_router, prefix="/auth", tags=["authentication"])
app.include_router(users_router, prefix="/api", tags=["users"])
app.include_router(tasks_router, prefix="/api", tags=["tasks"])
app.include_router(protected_router, prefix="/api", tags=["protected"])
app.include_router(chat_router, prefix="/api", tags=["chat"])


@app.get("/")
def read_root():
    return {
        "message": "Task Management API with Authentication",
        "endpoints": {
            "auth": "/auth/",
            "users": "/api/{user_id}/...",
            "protected": "/api/profile, /api/dashboard"
        }
    }


@app.get("/health")
def health_check():
    from utils.logging_config import logger
    logger.info("Health check endpoint called")
    return {
        "status": "healthy",
        "service": "task-management-auth-api",
        "timestamp": __import__('datetime').datetime.utcnow().isoformat()
    }