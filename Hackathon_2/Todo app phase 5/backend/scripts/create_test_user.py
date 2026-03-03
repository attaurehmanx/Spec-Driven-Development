"""
Create test user for running AI Agent Service tests
"""
from sqlmodel import Session, select
from database.session import engine
from models.task_models import User
from datetime import datetime

def create_test_user():
    """Create test user with ID 'test-user-123'"""
    with Session(engine) as session:
        # Check if test user already exists
        statement = select(User).where(User.id == 'test-user-123')
        existing_user = session.exec(statement).first()

        if existing_user:
            print('✓ Test user already exists')
            print(f'  ID: {existing_user.id}')
            print(f'  Email: {existing_user.email}')
            print(f'  Name: {existing_user.name}')
        else:
            # Create test user with required fields
            test_user = User(
                id='test-user-123',
                email='test@example.com',
                hashed_password='dummy_hash_for_testing',  # Not used in MCP tools
                name='Test User',
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                is_active=True,
                email_verified=True
            )
            session.add(test_user)
            session.commit()
            print('✓ Test user created successfully')
            print(f'  ID: {test_user.id}')
            print(f'  Email: {test_user.email}')
            print(f'  Name: {test_user.name}')

if __name__ == '__main__':
    create_test_user()
