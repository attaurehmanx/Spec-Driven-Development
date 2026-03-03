from fastapi import APIRouter, Depends
from api.deps import get_current_user_from_token
from models.task_models import UserResponse


router = APIRouter()


@router.get("/profile")
async def get_profile(current_user: UserResponse = Depends(get_current_user_from_token)):
    """Protected endpoint that returns user profile information."""
    return {
        "user": {
            "id": current_user.id,
            "email": current_user.email,
            "first_name": current_user.first_name,
            "last_name": current_user.last_name
        },
        "message": "This is a protected endpoint"
    }


@router.get("/dashboard")
async def get_dashboard_data(current_user: UserResponse = Depends(get_current_user_from_token)):
    """Protected endpoint that returns user dashboard data."""
    return {
        "user_id": current_user.id,
        "welcome_message": f"Welcome back, {current_user.first_name}!",
        "data": {
            "tasks_count": 0,  # This would come from DB in real implementation
            "completed_tasks": 0,  # This would come from DB in real implementation
            "recent_activity": []  # This would come from DB in real implementation
        }
    }


@router.post("/secure-action")
async def perform_secure_action(
    action_data: dict,
    current_user: UserResponse = Depends(get_current_user_from_token)
):
    """Protected endpoint that performs a secure action."""
    return {
        "success": True,
        "action": "secure-action",
        "user_id": current_user.id,
        "received_data": action_data,
        "message": "Action performed successfully with authentication"
    }