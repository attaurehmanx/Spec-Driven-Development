from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel import Session, select
from typing import List

from models.task_models import Task, TaskCreate, TaskUpdate, TaskResponse
from middleware.auth import get_current_user_with_validation
from database.session import get_session

router = APIRouter()


@router.get("/tasks", response_model=List[TaskResponse])
async def list_user_tasks(
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    List all tasks for the authenticated user.
    """
    # Query tasks for the authenticated user
    statement = select(Task).where(Task.user_id == current_user.id)
    tasks = session.exec(statement).all()

    return tasks


@router.post("/tasks", response_model=TaskResponse, status_code=status.HTTP_201_CREATED)
async def create_task(
    task_create: TaskCreate,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Create a new task for the authenticated user.
    """
    # Create new task with the authenticated user as owner
    task = Task(
        title=task_create.title,
        description=task_create.description,
        completed=task_create.completed,
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
    """
    # Query for the specific task owned by the authenticated user
    statement = select(Task).where(Task.id == task_id, Task.user_id == current_user.id)
    task = session.exec(statement).first()

    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Update task fields
    update_data = task_update.dict(exclude_unset=True)
    for field, value in update_data.items():
        setattr(task, field, value)

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


@router.patch("/tasks/{task_id}/complete", response_model=TaskResponse)
async def toggle_task_completion(
    task_id: int,
    current_user: TaskResponse = Depends(get_current_user_with_validation),
    session: Session = Depends(get_session)
):
    """
    Toggle the completion status of a specific task by ID for the authenticated user.
    """
    # Query for the specific task owned by the authenticated user
    statement = select(Task).where(Task.id == task_id, Task.user_id == current_user.id)
    task = session.exec(statement).first()

    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Toggle completion status
    task.completed = not task.completed

    session.add(task)
    session.commit()
    session.refresh(task)

    return task