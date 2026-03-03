"""
Verify database schema - check if columns exist
"""
import sys
from pathlib import Path

backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

from sqlalchemy import text, inspect
from database.session import engine
from config.settings import get_settings

def check_schema():
    """Check if the task table has the required columns"""
    settings = get_settings()
    print(f"Database URL: {settings.DATABASE_URL.split('@')[1] if '@' in settings.DATABASE_URL else 'configured'}")

    with engine.connect() as connection:
        # Get table columns
        inspector = inspect(engine)

        if 'task' in inspector.get_table_names():
            columns = inspector.get_columns('task')
            print("\nTask table columns:")
            for col in columns:
                print(f"  - {col['name']}: {col['type']}")

            # Check for specific columns
            column_names = [col['name'] for col in columns]
            required_columns = ['priority', 'tags', 'due_date', 'recurring']

            print("\nRequired columns check:")
            for req_col in required_columns:
                status = "✓ EXISTS" if req_col in column_names else "✗ MISSING"
                print(f"  {req_col}: {status}")
        else:
            print("Task table does not exist!")

if __name__ == "__main__":
    check_schema()
