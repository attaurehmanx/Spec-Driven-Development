from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlmodel import Session
from typing import Generator
from database.session import get_session
from utils.jwt import jwt_handler
from models.task_models import User
from utils.auth import TokenData
from sqlmodel import select


security = HTTPBearer()


def get_current_user_from_token(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    session: Session = Depends(get_session)
) -> User:
    """
    Dependency to extract and validate user from JWT token.
    This can be used to protect API endpoints.
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


def get_token_payload(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> TokenData:
    """
    Dependency to extract token payload without database lookup.
    Use this when you only need token information, not the full user object.
    """
    token = credentials.credentials
    token_data = jwt_handler.verify_token(token)

    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return token_data


def verify_token_exists(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> str:
    """
    Dependency to simply verify that a token exists and is valid.
    Returns the raw token if valid.
    """
    token = credentials.credentials
    token_data = jwt_handler.verify_token(token)

    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return token