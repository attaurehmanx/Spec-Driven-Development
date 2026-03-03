from fastapi import APIRouter, Depends, HTTPException, status, File, UploadFile
from sqlmodel import Session, select
from datetime import datetime
from models.task_models import User, UserResponse
from middleware.auth import get_current_user_with_validation
from database.session import get_session


router = APIRouter()


import os
from pathlib import Path

@router.get("/{user_id}/profile", response_model=UserResponse)
async def get_user_profile(
    user_id: str,
    current_user: UserResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Get user profile information.
    Validates that the user ID in the token matches the URL user ID.

    Note: On Hugging Face Spaces, avatar files may not persist due to ephemeral storage.
    If an avatar file is missing, the frontend should handle this gracefully.
    """
    # The authentication middleware already validates that the token user ID matches the URL user ID
    # So we can safely return the current user's profile

    # Create a copy of the current user to potentially modify the avatar field
    user_dict = current_user.dict()

    # If the user has an avatar and we're on Hugging Face (ephemeral storage),
    # we could check if the file exists and return a default if it doesn't
    # However, checking file existence for every profile request might be inefficient
    # Instead, the static file handler handles missing files gracefully
    # So we can just return the user as is, knowing the static file handler will serve
    # a default avatar if the requested file doesn't exist

    return UserResponse(**user_dict)


@router.get("/{user_id}/dashboard")
async def get_user_dashboard(
    user_id: str,
    current_user: UserResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Get user dashboard data.
    Validates that the user ID in the token matches the URL user ID.
    """
    # The authentication middleware already validates that the token user ID matches the URL user ID
    return {
        "user": {
            "id": current_user.id,
            "email": current_user.email,
            "first_name": current_user.first_name,
            "last_name": current_user.last_name
        },
        "dashboard": {
            "greeting": f"Hello, {current_user.first_name}!",
            "stats": {
                "tasks_count": 0,  # This would come from the database in a real implementation
                "completed_tasks": 0,  # This would come from the database in a real implementation
                "pending_tasks": 0  # This would come from the database in a real implementation
            }
        }
    }


@router.put("/{user_id}/profile")
async def update_user_profile(
    user_id: str,
    profile_data: dict,
    current_user: UserResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Update user profile information.
    Validates that the user ID in the token matches the URL user ID.
    """
    # The authentication middleware already validates that the token user ID matches the URL user ID

    # Update the user in the database
    statement = select(User).where(User.id == current_user.id)
    user = session.exec(statement).first()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    # Update user fields based on profile_data
    if "first_name" in profile_data and "last_name" in profile_data:
        # Combine first and last name into the single name field
        user.name = f"{profile_data['first_name']} {profile_data['last_name']}".strip()

    if "email" in profile_data:
        user.email = profile_data["email"]

    if "phone" in profile_data:
        user.phone = profile_data["phone"]

    if "address" in profile_data:
        user.address = profile_data["address"]

    if "avatar" in profile_data:
        user.avatar = profile_data["avatar"]

    # Update timestamps
    user.updated_at = datetime.utcnow()

    # Commit changes to database
    session.add(user)
    session.commit()
    session.refresh(user)

    # Return updated user data
    # Split the combined name back to first and last name for response
    name_parts = user.name.split(" ", 1)
    first_name = name_parts[0] if name_parts else ""
    last_name = name_parts[1] if len(name_parts) > 1 else "" if name_parts else ""

    updated_user_response = UserResponse(
        id=user.id,
        email=user.email,
        first_name=first_name,
        last_name=last_name,
        created_at=user.created_at,
        is_active=user.is_active,
        email_verified=user.email_verified,
        phone=user.phone,
        address=user.address,
        avatar=user.avatar
    )

    return {
        "success": True,
        "user": updated_user_response,
        "updated_fields": list(profile_data.keys()),
        "message": "Profile updated successfully"
    }


@router.post("/{user_id}/avatar")
async def upload_avatar(
    user_id: str,
    file: UploadFile = File(...),
    current_user: UserResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Upload user avatar/image.
    Validates that the user ID in the token matches the URL user ID.

    Note: On Hugging Face Spaces, file uploads are ephemeral and will not persist.
    For production use, consider using external storage (AWS S3, Google Cloud Storage, etc.).
    """
    # The authentication middleware already validates that the token user ID matches the URL user ID

    # Validate file type
    allowed_types = ["image/jpeg", "image/jpg", "image/png", "image/gif", "image/webp"]
    if file.content_type not in allowed_types:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid file type. Only image files are allowed."
        )

    # Read file content
    contents = await file.read()

    # You can either save the file to a static directory or to a cloud storage
    # For now, I'll save to a local directory (in a real app, you'd use cloud storage)
    import os
    import uuid
    from pathlib import Path

    # Create uploads directory if it doesn't exist
    upload_dir = Path("uploads/avatars")
    upload_dir.mkdir(parents=True, exist_ok=True)

    # Generate unique filename
    file_extension = Path(file.filename).suffix
    unique_filename = f"{current_user.id}_{uuid.uuid4()}{file_extension}"
    file_path = upload_dir / unique_filename

    try:
        # Save the file
        with open(file_path, "wb") as f:
            f.write(contents)

        # Update user's avatar field in the database
        statement = select(User).where(User.id == current_user.id)
        user = session.exec(statement).first()

        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        # Update avatar URL - in a real app, you'd serve these from a proper endpoint
        user.avatar = f"/api/uploads/avatars/{unique_filename}"
        user.updated_at = datetime.utcnow()

        session.add(user)
        session.commit()
        session.refresh(user)

        return {
            "success": True,
            "avatar_url": user.avatar,
            "message": "Avatar uploaded successfully (Note: On Hugging Face Spaces, avatars may not persist between deployments. Consider using external storage for production.)"
        }
    except Exception as e:
        # If file saving fails (common on Hugging Face due to ephemeral storage),
        # we can still update the user record with a default avatar
        statement = select(User).where(User.id == current_user.id)
        user = session.exec(statement).first()

        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        # Set to default avatar instead
        user.avatar = "/api/uploads/avatars/default-avatar.png"
        user.updated_at = datetime.utcnow()

        session.add(user)
        session.commit()
        session.refresh(user)

        return {
            "success": False,
            "avatar_url": user.avatar,
            "message": f"Failed to save avatar: {str(e)}. Using default avatar instead. On Hugging Face Spaces, file uploads are ephemeral. Consider using external storage for production."
        }


@router.get("/avatars/{filename}")
async def get_avatar(filename: str):
    """
    Serve avatar files with fallback to default avatar.
    This endpoint handles the ephemeral storage issue on Hugging Face Spaces.
    """
    import os
    from pathlib import Path
    from starlette.responses import FileResponse, PlainTextResponse
    import mimetypes

    # Define the path to the avatar file
    avatar_path = Path("uploads/avatars") / filename

    # Check if the requested avatar file exists
    if avatar_path.exists():
        # Serve the requested avatar file
        media_type, _ = mimetypes.guess_type(str(avatar_path))
        return FileResponse(
            path=str(avatar_path),
            media_type=media_type or "application/octet-stream"
        )

    # If the requested avatar doesn't exist, serve the default avatar
    default_avatar_paths = [
        Path("uploads/avatars/default-avatar.png"),
        Path("uploads/avatars/default-avatar.svg")
    ]

    for default_path in default_avatar_paths:
        if default_path.exists():
            media_type, _ = mimetypes.guess_type(str(default_path))
            return FileResponse(
                path=str(default_path),
                media_type=media_type or "image/svg+xml"  # SVG as default for avatar
            )

    # If no default avatar exists, return a 404
    return PlainTextResponse("Avatar not found", status_code=404)


@router.delete("/{user_id}")
async def delete_user_account(
    user_id: str,
    current_user: UserResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Delete user account.
    Validates that the user ID in the token matches the URL user ID.
    """
    # The authentication middleware already validates that the token user ID matches the URL user ID

    # Find the user in the database
    statement = select(User).where(User.id == current_user.id)
    user = session.exec(statement).first()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    # Delete the user's tasks first to avoid foreign key constraint issues
    from models.task_models import Task
    task_statement = select(Task).where(Task.user_id == current_user.id)
    user_tasks = session.exec(task_statement).all()

    for task in user_tasks:
        session.delete(task)

    # Delete the user from the database
    session.delete(user)
    session.commit()

    return {
        "success": True,
        "user_id": current_user.id,
        "message": "Account deleted successfully"
    }