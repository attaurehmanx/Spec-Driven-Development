from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlmodel import Session, select
from typing import List, Optional
from pydantic import BaseModel
from datetime import datetime

from models.task_models import Task, TaskCreate, TaskUpdate, TaskResponse, PriorityLevel, RecurringPattern
from middleware.auth import get_current_user_with_validation
from database.session import get_session
from services.task_service import TaskService
from services.recurring_service import RecurringService

router = APIRouter()


# Custom response model for task completion with optional next instance
class TaskCompletionResponse(BaseModel):
    """Response model for task completion that includes the completed task and optional next instance."""
    task: TaskResponse
    next_instance: Optional[TaskResponse] = None

    class Config:
        from_attributes = True


@router.get("/tasks", response_model=List[TaskResponse])
async def list_user_tasks(
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session),
    priority: Optional[PriorityLevel] = Query(None, description="Filter by priority level"),
    tags: Optional[str] = Query(None, description="Filter by tags (comma-separated)"),
    status: Optional[str] = Query(None, description="Filter by status (done/not_done)"),
    search: Optional[str] = Query(None, description="Search in title and description"),
    sort: Optional[str] = Query(None, description="Sort by: priority, due_date, created_at, title"),
    limit: int = Query(1000, ge=1, le=1000, description="Maximum number of tasks to return (1-1000)"),
    offset: int = Query(0, ge=0, description="Number of tasks to skip for pagination")
):
    """
    List all tasks for the authenticated user with optional filtering, sorting, and pagination.

    Query Parameters:
    - priority: Filter by priority level (high, medium, low)
    - tags: Filter by tags (comma-separated, e.g., "work,urgent")
    - status: Filter by completion status (done, not_done)
    - search: Search in title and description (case-insensitive)
    - sort: Sort by priority, due_date, created_at, or title
    - limit: Maximum number of tasks to return (default: 1000, max: 1000)
    - offset: Number of tasks to skip for pagination (default: 0)
    """
    # Parse tags from comma-separated string
    tags_list = tags.split(",") if tags else None

    # Filter tasks using service layer
    tasks = TaskService.filter_tasks(
        session=session,
        user_id=current_user.id,
        priority=priority,
        tags=tags_list,
        status=status,
        search=search
    )

    # Sort tasks if sort parameter provided
    if sort:
        tasks = TaskService.sort_tasks(tasks, sort_by=sort)

    # Apply pagination
    paginated_tasks = tasks[offset:offset + limit]

    return paginated_tasks


@router.post("/tasks", response_model=TaskResponse, status_code=status.HTTP_201_CREATED)
async def create_task(
    task_create: TaskCreate,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Create a new task for the authenticated user.

    Accepts all task fields including priority, tags, due_date, and recurring pattern.
    """
    # Create new task with all fields from TaskCreate schema
    task = Task(
        title=task_create.title,
        description=task_create.description,
        completed=task_create.completed,
        priority=task_create.priority,
        tags=task_create.tags,
        due_date=task_create.due_date,
        recurring=task_create.recurring,
        user_id=current_user.id
    )

    session.add(task)
    session.commit()
    session.refresh(task)

    return task


@router.get("/tasks/{task_id}", response_model=TaskResponse)
async def get_task(
    task_id: int,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Get a specific task by ID for the authenticated user.
    """
    # Query for the specific task owned by the authenticated user
    statement = select(Task).where(Task.id == task_id, Task.user_id == current_user.id)
    task = session.exec(statement).first()

    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    return task


@router.put("/tasks/{task_id}", response_model=TaskResponse)
async def update_task(
    task_id: int,
    task_update: TaskUpdate,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Update a specific task by ID for the authenticated user.

    Accepts all task fields including priority, tags, due_date, and recurring pattern.
    """
    # Query for the specific task owned by the authenticated user
    statement = select(Task).where(Task.id == task_id, Task.user_id == current_user.id)
    task = session.exec(statement).first()

    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Update task fields (only fields provided in request)
    update_data = task_update.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(task, field, value)

    # Update the updated_at timestamp
    from datetime import datetime
    task.updated_at = datetime.utcnow()

    session.add(task)
    session.commit()
    session.refresh(task)

    return task


@router.delete("/tasks/{task_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_task(
    task_id: int,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Delete a specific task by ID for the authenticated user.
    """
    # Query for the specific task owned by the authenticated user
    statement = select(Task).where(Task.id == task_id, Task.user_id == current_user.id)
    task = session.exec(statement).first()

    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    session.delete(task)
    session.commit()

    return


@router.patch("/tasks/{task_id}/complete", response_model=TaskCompletionResponse)
async def toggle_task_completion(
    task_id: int,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Toggle the completion status of a specific task by ID for the authenticated user.

    If the task is recurring and being marked as completed, automatically creates
    the next instance with the calculated due date.

    Returns:
        TaskCompletionResponse containing the completed task and optional next_instance
    """
    # Query for the specific task owned by the authenticated user
    statement = select(Task).where(Task.id == task_id, Task.user_id == current_user.id)
    task = session.exec(statement).first()

    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Store the original recurring pattern before toggling
    was_recurring = task.recurring != RecurringPattern.NONE
    was_completed = task.completed

    # Toggle completion status
    task.completed = not task.completed

    session.add(task)
    session.commit()
    session.refresh(task)

    # Create next instance if this is a recurring task being marked as completed
    next_instance = None
    if was_recurring and not was_completed and task.completed:
        # Task was just marked as completed and is recurring
        next_task_data = RecurringService.create_next_instance(task, current_user.id)

        # Create the new task instance
        next_task = Task(
            user_id=current_user.id,
            title=next_task_data.title,
            description=next_task_data.description,
            completed=next_task_data.completed,
            priority=next_task_data.priority,
            tags=next_task_data.tags,
            due_date=next_task_data.due_date,
            recurring=next_task_data.recurring,
            parent_task_id=next_task_data.parent_task_id
        )

        session.add(next_task)
        session.commit()
        session.refresh(next_task)

        next_instance = next_task

    return TaskCompletionResponse(
        task=task,
        next_instance=next_instance
    )


@router.get("/tasks/tags", response_model=List[str])
async def get_user_tags(
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Get all unique tags used by the authenticated user across all their tasks.

    Returns a list of unique tag strings sorted alphabetically.
    """
    # Query all tasks for the authenticated user
    statement = select(Task).where(Task.user_id == current_user.id)
    tasks = session.exec(statement).all()

    # Collect all unique tags
    unique_tags = set()
    for task in tasks:
        if task.tags:
            unique_tags.update(task.tags)

    # Return sorted list of unique tags
    return sorted(list(unique_tags))


@router.get("/tasks/overdue", response_model=List[TaskResponse])
async def get_overdue_tasks(
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Get all overdue tasks for the authenticated user.

    Returns tasks where due_date < NOW() AND completed = false,
    sorted by due date (most overdue first).
    """
    # Get overdue tasks using service layer
    overdue_tasks = TaskService.get_overdue_tasks(
        session=session,
        user_id=current_user.id
    )

    # Sort by due date (most overdue first)
    sorted_tasks = sorted(overdue_tasks, key=lambda t: t.due_date if t.due_date else datetime.max)

    return sorted_tasks
    for task in tasks:
        if task.tags:
            unique_tags.update(task.tags)

    # Return sorted list of unique tags
    return sorted(list(unique_tags))