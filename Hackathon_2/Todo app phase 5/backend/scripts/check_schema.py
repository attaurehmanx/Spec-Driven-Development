"""Check database schema for message table"""
import sys
import os
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

from database.session import engine
from sqlalchemy import text, inspect

print("Checking message table schema...")
print("=" * 70)

with engine.connect() as conn:
    # Check if message table exists
    inspector = inspect(engine)
    tables = inspector.get_table_names()
    print(f"Tables in database: {tables}")

    if 'message' in tables:
        print("\nMessage table columns:")
        columns = inspector.get_columns('message')
        for col in columns:
            print(f"  - {col['name']}: {col['type']} (nullable={col['nullable']})")

        # Check for enum types
        result = conn.execute(text("""
            SELECT typname, enumlabel
            FROM pg_type
            JOIN pg_enum ON pg_type.oid = pg_enum.enumtypid
            WHERE typname LIKE '%message%'
            ORDER BY typname, enumsortorder
        """))

        enum_values = list(result)
        if enum_values:
            print("\nPostgreSQL enum types found:")
            current_type = None
            for typename, label in enum_values:
                if typename != current_type:
                    print(f"\n  {typename}:")
                    current_type = typename
                print(f"    - {label}")
        else:
            print("\nNo PostgreSQL enum types found for message table")
    else:
        print("\nMessage table does not exist!")

    # Check conversation table
    if 'conversation' in tables:
        print("\n" + "=" * 70)
        print("Conversation table columns:")
        columns = inspector.get_columns('conversation')
        for col in columns:
            print(f"  - {col['name']}: {col['type']} (nullable={col['nullable']})")
