"""
Test script to verify async database connection works correctly.
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from database.session import get_async_session
from models.task_models import User
from sqlmodel import select


async def test_async_connection():
    """Test async database connection by running a simple query."""
    print("Testing async database connection...")

    try:
        # Get async session
        async for session in get_async_session():
            # Run a simple query to verify connection
            stmt = select(User).limit(1)
            result = await session.execute(stmt)
            users = result.scalars().all()

            print(f"[OK] Async database connection successful!")
            print(f"[OK] Found {len(users)} user(s) in database")

            if users:
                print(f"[OK] Sample user: {users[0].email}")

            return True
    except Exception as e:
        print(f"[ERROR] Async database connection failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(test_async_connection())
    sys.exit(0 if success else 1)
