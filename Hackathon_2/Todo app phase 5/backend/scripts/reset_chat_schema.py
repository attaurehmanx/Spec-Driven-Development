"""
Reset chat database schema (drop and recreate conversation and message tables)
This fixes the enum issue and ensures clean schema
"""
import sys
import os
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

from database.session import engine
from sqlalchemy import text

print("Resetting chat database schema...")
print("=" * 70)

with engine.connect() as conn:
    # Drop tables in correct order (message first due to foreign key)
    print("Dropping message table...")
    conn.execute(text("DROP TABLE IF EXISTS message CASCADE"))
    conn.commit()

    print("Dropping conversation table...")
    conn.execute(text("DROP TABLE IF EXISTS conversation CASCADE"))
    conn.commit()

    # Drop the enum type
    print("Dropping messagerole enum type...")
    conn.execute(text("DROP TYPE IF EXISTS messagerole CASCADE"))
    conn.commit()

    print("\n[PASS] Old schema dropped successfully")

print("\nRecreating tables with correct schema...")

# Import models to register them with SQLModel metadata
from sqlmodel import SQLModel
from models.conversation import Conversation
from models.message import Message, MessageRole

# Create tables
SQLModel.metadata.create_all(bind=engine)

print("[PASS] New schema created successfully")

# Verify the schema
print("\nVerifying schema...")
with engine.connect() as conn:
    # Check enum values
    result = conn.execute(text("""
        SELECT enumlabel
        FROM pg_enum
        WHERE enumtypid = (SELECT oid FROM pg_type WHERE typname = 'messagerole')
        ORDER BY enumsortorder
    """))

    enum_values = [row[0] for row in result]
    print(f"MessageRole enum values: {enum_values}")

    expected = ['user', 'assistant', 'system']
    if enum_values == expected:
        print("[PASS] Enum values are correct")
    else:
        print(f"[FAIL] Expected {expected}, got {enum_values}")

print("\n" + "=" * 70)
print("[SUCCESS] Chat schema reset complete!")
print("\nYou can now run: python test_phase3_us1.py")
