from sqlmodel import create_engine, Session
from typing import Generator
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from config.settings import get_settings
import logging

# Get database URL from settings
settings = get_settings()
database_url = settings.DATABASE_URL

# Create the database engine with connection pooling
engine = create_engine(
    database_url,
    echo=False,  # Set to True for SQL query logging
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,  # Recycle connections after 5 minutes
    pool_size=10,  # Number of connections to maintain in the pool
    max_overflow=20,  # Additional connections beyond pool_size
    pool_timeout=30,  # Timeout for getting a connection from the pool
    pool_reset_on_return='commit',  # Reset connection on return to pool
)

def get_session() -> Generator[Session, None, None]:
    """
    Dependency to get a database session with proper connection pooling
    """
    with Session(engine) as session:
        try:
            yield session
        except Exception as e:
            # Log the exception
            logging.error(f"Database session error: {str(e)}")
            session.rollback()
            raise