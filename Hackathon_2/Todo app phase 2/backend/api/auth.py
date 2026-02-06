from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import Session
from datetime import timedelta, datetime
from models.task_models import User, UserCreate, UserResponse, Token, LoginRequest, RefreshTokenRequest
from utils.auth import authenticate_user, create_access_token, get_password_hash, create_refresh_token, verify_refresh_token, TokenData
from database.session import get_session
from config.settings import get_settings
settings = get_settings()
from services.user_service import create_user, get_user_by_email
from middleware.auth import get_current_user
import sys
import os
# Add the backend root directory to the path to access utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from utils.logging_config import logger


router = APIRouter()


@router.post("/register", response_model=UserResponse)
async def register(user: UserCreate, session: Session = Depends(get_session)):
    logger.info(f"Registration attempt for user: {user.email}")

    # Check if user already exists
    existing_user = await get_user_by_email(session, user.email)
    if existing_user:
        logger.warning(f"Registration failed - user already exists: {user.email}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="User with this email already exists"
        )

    # Create new user
    created_user = await create_user(session, user)
    logger.info(f"Successfully registered user: {user.email} (ID: {created_user.id})")

    return created_user


@router.post("/login", response_model=Token)
async def login(login_request: LoginRequest, session: Session = Depends(get_session)):
    email = login_request.email
    password = login_request.password
    logger.info(f"Login attempt for user: {email}")

    user = authenticate_user(session, email, password)
    if not user:
        logger.warning(f"Failed login attempt for user: {email}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": str(user.id)}, expires_delta=access_token_expires
    )

    # Create refresh token with longer expiration (7 days)
    refresh_token = create_refresh_token(
        data={"sub": str(user.id)}
    )

    logger.info(f"Successful login for user: {email} (ID: {user.id})")

    return {
        "access_token": access_token,
        "refresh_token": refresh_token,
        "token_type": "bearer"
    }


@router.post("/refresh", response_model=Token)
async def refresh_token(refresh_request: RefreshTokenRequest):
    """Endpoint to refresh access token using refresh token."""
    logger.info("Refresh token request received")

    token_data = verify_refresh_token(refresh_request.refresh_token)
    if not token_data:
        logger.warning("Invalid or expired refresh token")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired refresh token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Create new access token
    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": token_data.sub}, expires_delta=access_token_expires
    )

    # Create new refresh token (rolling refresh)
    refresh_token_new = create_refresh_token(
        data={"sub": token_data.sub}
    )

    logger.info(f"Successfully refreshed tokens for user: {token_data.sub}")

    return {
        "access_token": access_token,
        "refresh_token": refresh_token_new,
        "token_type": "bearer"
    }


@router.get("/verify-token")
async def verify_token(current_user: User = Depends(get_current_user)):  # Changed to User instead of UserResponse
    """Protected endpoint to verify if a token is valid and return user info."""
    logger.info(f"Token verification successful for user: {current_user.email} (ID: {current_user.id})")

    # Split the combined name into first and last name
    name_parts = current_user.name.split(" ", 1)
    first_name = name_parts[0] if name_parts else ""
    last_name = name_parts[1] if len(name_parts) > 1 else ""

    return {
        "authenticated": True,
        "user": {
            "id": current_user.id,
            "email": current_user.email,
            "first_name": first_name,
            "last_name": last_name,
            "phone": getattr(current_user, 'phone', None),
            "address": getattr(current_user, 'address', None),
            "avatar": getattr(current_user, 'avatar', None)
        }
    }