"""Verify database indexes for conversation and message tables"""
import sys
import os
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

from database.session import engine
from sqlalchemy import inspect

print("Verifying Database Indexes")
print("=" * 70)

inspector = inspect(engine)

# Check conversation table indexes
print("\nConversation Table Indexes:")
print("-" * 70)
conv_indexes = inspector.get_indexes('conversation')
for idx in conv_indexes:
    columns = ', '.join(idx['column_names'])
    print(f"  - {idx['name']}: ({columns}) [unique={idx['unique']}]")

expected_conv_indexes = ['user_id', 'updated_at']
found_conv_indexes = []
for idx in conv_indexes:
    for col in idx['column_names']:
        if col in expected_conv_indexes:
            found_conv_indexes.append(col)

print(f"\nExpected indexes: {expected_conv_indexes}")
print(f"Found indexes on: {list(set(found_conv_indexes))}")

# Check message table indexes
print("\n" + "=" * 70)
print("Message Table Indexes:")
print("-" * 70)
msg_indexes = inspector.get_indexes('message')
for idx in msg_indexes:
    columns = ', '.join(idx['column_names'])
    print(f"  - {idx['name']}: ({columns}) [unique={idx['unique']}]")

expected_msg_indexes = ['conversation_id', 'created_at']
found_msg_indexes = []
for idx in msg_indexes:
    for col in idx['column_names']:
        if col in expected_msg_indexes:
            found_msg_indexes.append(col)

print(f"\nExpected indexes: {expected_msg_indexes}")
print(f"Found indexes on: {list(set(found_msg_indexes))}")

# Summary
print("\n" + "=" * 70)
print("Index Verification Summary")
print("=" * 70)

all_expected = set(expected_conv_indexes + expected_msg_indexes)
all_found = set(found_conv_indexes + found_msg_indexes)

if all_expected.issubset(all_found):
    print("[SUCCESS] All required indexes are present!")
else:
    missing = all_expected - all_found
    print(f"[WARNING] Missing indexes: {missing}")
