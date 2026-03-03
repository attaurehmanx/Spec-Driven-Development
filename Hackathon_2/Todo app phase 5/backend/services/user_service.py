from sqlmodel import Session, select
from models.task_models import User, UserCreate, UserResponse
from typing import Optional
from utils.auth import get_password_hash
from utils.validation import UserValidation


async def create_user(session: Session, user: UserCreate) -> UserResponse:
    """Create a new user"""
    # Validate that required fields are present
    if not hasattr(user, 'password') or not user.password:
        raise ValueError("Password is required")
    if not hasattr(user, 'first_name') or not user.first_name:
        raise ValueError("First name is required")
    if not hasattr(user, 'last_name') or not user.last_name:
        raise ValueError("Last name is required")
    if not hasattr(user, 'email') or not user.email:
        raise ValueError("Email is required")

    # Validate password strength before hashing
    if len(user.password) < 8:
        raise ValueError("Password must be at least 8 characters long")

    if not UserValidation.validate_email(user.email):
        raise ValueError("Invalid email format")

    # Hash the password
    hashed_password = get_password_hash(user.password)
    if not hashed_password:
        raise ValueError("Failed to hash password")

    # Combine first_name and last_name into the name field that the database expects
    full_name = f"{user.first_name} {user.last_name}".strip()
    if not full_name:
        raise ValueError("Full name could not be created")
    db_user = User(
        email=user.email,
        hashed_password=hashed_password,
        name=full_name
    )
    session.add(db_user)
    session.commit()
    session.refresh(db_user)

    # Split the full name back into first and last name for response
    name_parts = db_user.name.split(" ", 1)
    first_name = name_parts[0] if name_parts else ""
    last_name = name_parts[1] if len(name_parts) > 1 else ""

    return UserResponse(
        id=db_user.id,
        email=db_user.email,
        first_name=first_name,
        last_name=last_name,
        created_at=db_user.created_at
    )


async def get_user_by_email(session: Session, email: str) -> Optional[User]:
    """Get a user by email"""
    statement = select(User).where(User.email == email)
    return session.exec(statement).first()


async def get_user_by_id(session: Session, user_id: str) -> Optional[User]:
    """Get a user by ID"""
    statement = select(User).where(User.id == user_id)
    return session.exec(statement).first()