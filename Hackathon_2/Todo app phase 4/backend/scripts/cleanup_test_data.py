"""Clean up test data from database"""
import sys
import os
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

from database.session import get_session
from sqlalchemy import text

print("Cleaning up test data...")
print("=" * 70)

with next(get_session()) as session:
    # Delete in correct order: messages, conversations, then users

    # Delete messages from test conversations
    result1 = session.execute(text("""
        DELETE FROM message
        WHERE conversation_id IN (
            SELECT id FROM conversation
            WHERE user_id LIKE 'test-user-phase3-%'
            OR user_id LIKE 'perf-test-user%'
        )
    """))
    session.commit()
    print(f"[PASS] Deleted {result1.rowcount} test messages")

    # Delete conversations
    result2 = session.execute(text("""
        DELETE FROM conversation
        WHERE user_id LIKE 'test-user-phase3-%'
        OR user_id LIKE 'perf-test-user%'
    """))
    session.commit()
    print(f"[PASS] Deleted {result2.rowcount} test conversations")

    # Delete test users
    result3 = session.execute(text("""
        DELETE FROM "user"
        WHERE id LIKE 'test-user-phase3-%'
        OR id LIKE 'perf-test-user%'
    """))
    session.commit()
    print(f"[PASS] Deleted {result3.rowcount} test users")

print("=" * 70)
print("[SUCCESS] Cleanup complete!")
