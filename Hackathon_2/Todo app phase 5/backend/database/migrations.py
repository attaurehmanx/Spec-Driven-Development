import sys
import os

# Add backend directory to Python path when running as script
if __name__ == "__main__":
    backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, backend_dir)

from sqlmodel import SQLModel
from database.session import engine
# Import all models to ensure they're registered with SQLModel metadata
from models.task_models import User, Task
from models.conversation import Conversation
from models.message import Message


def create_db_and_tables():
    """
    Create database tables based on SQLModel definitions
    This should be called on application startup
    """
    SQLModel.metadata.create_all(bind=engine)


if __name__ == "__main__":
    # For development/testing purposes
    create_db_and_tables()
    print("Database tables created successfully!")