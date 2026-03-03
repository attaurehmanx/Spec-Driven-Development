from fastapi import HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlmodel import Session, select
from typing import Optional
from models.task_models import User
from utils.auth import TokenData
from database.session import get_session
from utils.jwt import jwt_handler
from utils.user_identity import user_identity_extractor


security = HTTPBearer()


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
) -> User:
    """
    Dependency to get the current user from the JWT token.
    This will be used to protect API endpoints.
    """
    token = credentials.credentials
    token_data = jwt_handler.verify_token(token)

    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Find user by ID from token - explicitly select all fields including phone and address
    statement = select(User).where(User.id == token_data.user_id)
    user = session.exec(statement).first()

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Explicitly refresh the user object to ensure all fields are loaded
    session.refresh(user)

    # Debug logging to see what fields are available
    from utils.logging_config import logger
    logger.info(f"Retrieved user from DB: id={user.id}, email={user.email}")
    logger.info(f"User object type: {type(user)}")
    logger.info(f"User has phone attribute: {hasattr(user, 'phone')}")
    logger.info(f"User has address attribute: {hasattr(user, 'address')}")
    logger.info(f"User phone value: {getattr(user, 'phone', 'PHONE_NOT_FOUND')}")
    logger.info(f"User address value: {getattr(user, 'address', 'ADDRESS_NOT_FOUND')}")

    return user


def verify_user_id_match(token_user_id: str, url_user_id: str) -> bool:
    """
    Verify that the user ID in the JWT token matches the user ID in the URL parameter.
    This ensures users can only access their own data.
    """
    return user_identity_extractor.validate_user_identity_match(token_user_id, url_user_id)


async def get_current_user_with_validation(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
) -> User:
    """
    Dependency to get the current user from the JWT token.
    """
    token = credentials.credentials
    token_data = jwt_handler.verify_token(token)

    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Find user by ID from token
    statement = select(User).where(User.id == token_data.user_id)
    user = session.exec(statement).first()

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user


async def validate_user_identity_and_permissions(
    token_user_id: str,
    required_user_id: str,
    session: Session
) -> tuple[bool, Optional[User]]:
    """
    Validate user identity and permissions in one function.
    Returns: (is_valid, user_object_if_valid)
    """
    # Validate that token user ID matches required user ID
    is_valid = user_identity_extractor.validate_user_identity_match(token_user_id, required_user_id)

    if not is_valid:
        return False, None

    # Find user in database
    statement = select(User).where(User.id == token_user_id)
    user = session.exec(statement).first()

    return True, user