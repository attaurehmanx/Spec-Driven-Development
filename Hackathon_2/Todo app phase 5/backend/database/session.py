from sqlmodel import create_engine, Session
from sqlmodel.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker
from typing import Generator, AsyncGenerator
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from config.settings import get_settings
import logging

# Get database URL from settings
settings = get_settings()
database_url = settings.DATABASE_URL

# Create the synchronous database engine with connection pooling (for FastAPI)
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

# Create the async database engine with asyncpg driver (for MCP server)
# Convert postgresql:// to postgresql+asyncpg:// and remove sslmode parameter
# asyncpg doesn't support sslmode parameter - it uses connect_args instead
async_database_url = database_url.replace('postgresql://', 'postgresql+asyncpg://')
# Remove sslmode parameter if present (asyncpg doesn't support it in URL)
if '?sslmode=' in async_database_url:
    async_database_url = async_database_url.split('?sslmode=')[0]
elif '&sslmode=' in async_database_url:
    async_database_url = async_database_url.replace('&sslmode=require', '').replace('&sslmode=prefer', '')

async_engine = create_async_engine(
    async_database_url,
    echo=False,  # Set to True for SQL query logging
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,  # Recycle connections after 5 minutes
    pool_size=10,  # Number of connections to maintain in the pool
    max_overflow=20,  # Additional connections beyond pool_size
    pool_timeout=30,  # Timeout for getting a connection from the pool
    connect_args={"ssl": "require"} if "neon.tech" in database_url else {},  # SSL for Neon
)

# Create async session maker
async_session_maker = sessionmaker(
    bind=async_engine,
    class_=AsyncSession,
    expire_on_commit=False,
)

def get_session() -> Generator[Session, None, None]:
    """
    Dependency to get a database session with proper connection pooling (synchronous)
    """
    with Session(engine) as session:
        try:
            yield session
        except Exception as e:
            # Log the exception
            logging.error(f"Database session error: {str(e)}")
            session.rollback()
            raise

async def get_async_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency to get an async database session with proper connection pooling
    """
    async with async_session_maker() as session:
        try:
            yield session
        except Exception as e:
            # Log the exception
            logging.error(f"Async database session error: {str(e)}")
            await session.rollback()
            raise