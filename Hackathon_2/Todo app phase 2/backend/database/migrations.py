from sqlmodel import SQLModel
from .session import engine


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