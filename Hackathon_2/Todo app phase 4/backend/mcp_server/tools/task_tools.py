"""
Task Management Tools for MCP Server

This module implements 5 task management tools:
1. add_task - Create new tasks
2. list_tasks - Retrieve user's task list
3. complete_task - Mark tasks as completed
4. update_task - Update task details
5. delete_task - Delete tasks

All tools enforce user isolation and validate ownership before operations.
"""

import logging
from datetime import datetime
from typing import Optional, Literal
from pydantic import BaseModel, Field, field_validator, model_validator
from sqlmodel import select

# Import MCP server instance
from mcp_server.server import mcp

# Import database models and session
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from models.task_models import Task, User
from database.session import get_async_session

logger = logging.getLogger(__name__)


# ============================================================================
# Custom Exception Classes
# ============================================================================

class TaskToolError(Exception):
    """Base exception for task tool errors."""
    pass


class TaskNotFoundError(TaskToolError):
    """Task does not exist."""
    pass


class UnauthorizedTaskAccessError(TaskToolError):
    """User does not own the task."""
    pass


class ValidationError(TaskToolError):
    """Input validation failed."""
    pass


# ============================================================================
# Input Schema Models (Pydantic)
# ============================================================================

class AddTaskInput(BaseModel):
    """Input schema for creating a new task."""
    user_id: str = Field(..., description="Authenticated user's UUID")
    title: str = Field(..., min_length=1, max_length=255, description="Task title")
    description: Optional[str] = Field(None, max_length=10000, description="Optional task description")

    @field_validator('title')
    @classmethod
    def title_not_empty(cls, v):
        if not v or not v.strip():
            raise ValueError("Title cannot be empty or whitespace")
        return v.strip()


class ListTasksInput(BaseModel):
    """Input schema for retrieving user's task list."""
    user_id: str = Field(..., description="Authenticated user's UUID")
    status: Literal["all", "pending", "completed"] = Field(
        default="all",
        description="Filter tasks by completion status"
    )


class CompleteTaskInput(BaseModel):
    """Input schema for marking a task as completed."""
    user_id: str = Field(..., description="Authenticated user's UUID")
    task_id: int = Field(..., gt=0, description="Task ID to mark as completed")


class UpdateTaskInput(BaseModel):
    """Input schema for updating task details."""
    user_id: str = Field(..., description="Authenticated user's UUID")
    task_id: int = Field(..., gt=0, description="Task ID to update")
    title: Optional[str] = Field(None, min_length=1, max_length=255, description="New task title")
    description: Optional[str] = Field(None, max_length=10000, description="New task description")

    @field_validator('title')
    @classmethod
    def title_not_empty(cls, v):
        if v is not None and (not v or not v.strip()):
            raise ValueError("Title cannot be empty or whitespace")
        return v.strip() if v else None

    @model_validator(mode='after')
    def at_least_one_field(self):
        if self.title is None and self.description is None:
            raise ValueError("At least one of title or description must be provided")
        return self


class DeleteTaskInput(BaseModel):
    """Input schema for deleting a task."""
    user_id: str = Field(..., description="Authenticated user's UUID")
    task_id: int = Field(..., gt=0, description="Task ID to delete")


# ============================================================================
# Tool Implementations
# ============================================================================

@mcp.tool()
async def add_task(
    user_id: str,
    title: str,
    description: str = None
) -> dict:
    """Create a new task for the user.

    Args:
        user_id: The authenticated user's UUID
        title: Task title (1-255 characters)
        description: Optional task description (max 10,000 characters)

    Returns:
        dict: Task creation result with task_id, status, and task details

    Raises:
        ValueError: If user not found or validation fails
    """
    logger.info(f"add_task called: user_id={user_id}, title={title[:50]}...")

    try:
        # Validate input using Pydantic model
        input_data = AddTaskInput(
            user_id=user_id,
            title=title,
            description=description
        )

        # Get async database session
        async for session in get_async_session():
            # Verify user exists
            user = await session.get(User, input_data.user_id)
            if not user:
                logger.warning(f"User not found: {input_data.user_id}")
                raise ValueError(f"User {input_data.user_id} not found")

            # Create new task
            new_task = Task(
                user_id=input_data.user_id,
                title=input_data.title,
                description=input_data.description,
                completed=False,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow()
            )

            # Persist to database
            session.add(new_task)
            await session.commit()
            await session.refresh(new_task)

            # Format response
            response = {
                "task_id": new_task.id,
                "status": "created",
                "title": new_task.title,
                "description": new_task.description,
                "completed": new_task.completed,
                "created_at": new_task.created_at.isoformat() + "Z",
                "updated_at": new_task.updated_at.isoformat() + "Z"
            }

            logger.info(f"Task created successfully: task_id={new_task.id}")
            return response

    except ValueError as e:
        logger.error(f"Validation error in add_task: {str(e)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in add_task: {str(e)}")
        raise TaskToolError(f"Failed to create task: {str(e)}")


@mcp.tool()
async def list_tasks(
    user_id: str,
    status: str = "all"
) -> dict:
    """List tasks for a user with optional status filter.

    Args:
        user_id: The authenticated user's UUID
        status: Filter by status - "all", "pending", or "completed"

    Returns:
        dict: List of tasks with count and filter information

    Raises:
        ValueError: If user not found or invalid status filter
    """
    logger.info(f"list_tasks called: user_id={user_id}, status={status}")

    try:
        # Validate input using Pydantic model
        input_data = ListTasksInput(
            user_id=user_id,
            status=status
        )

        # Get async database session
        async for session in get_async_session():
            # Verify user exists
            user = await session.get(User, input_data.user_id)
            if not user:
                logger.warning(f"User not found: {input_data.user_id}")
                raise ValueError(f"User {input_data.user_id} not found")

            # Build query based on status filter
            stmt = select(Task).where(Task.user_id == input_data.user_id)

            if input_data.status == "pending":
                stmt = stmt.where(Task.completed == False)
            elif input_data.status == "completed":
                stmt = stmt.where(Task.completed == True)
            # "all" doesn't need additional filtering

            # Order by created_at descending (newest first)
            stmt = stmt.order_by(Task.created_at.desc())

            # Execute query
            result = await session.execute(stmt)
            tasks = result.scalars().all()

            # Format response
            task_list = []
            for task in tasks:
                task_list.append({
                    "task_id": task.id,
                    "title": task.title,
                    "description": task.description,
                    "completed": task.completed,
                    "created_at": task.created_at.isoformat() + "Z",
                    "updated_at": task.updated_at.isoformat() + "Z"
                })

            response = {
                "tasks": task_list,
                "count": len(task_list),
                "filter": input_data.status
            }

            logger.info(f"Retrieved {len(task_list)} tasks for user {input_data.user_id} with filter '{input_data.status}'")
            return response

    except ValueError as e:
        logger.error(f"Validation error in list_tasks: {str(e)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in list_tasks: {str(e)}")
        raise TaskToolError(f"Failed to list tasks: {str(e)}")


@mcp.tool()
async def complete_task(
    user_id: str,
    task_id: int
) -> dict:
    """Mark a task as completed.

    Args:
        user_id: The authenticated user's UUID
        task_id: The task ID to mark as completed

    Returns:
        dict: Updated task details with status="completed"

    Raises:
        ValueError: If user not found
        TaskNotFoundError: If task not found
        UnauthorizedTaskAccessError: If user doesn't own the task
    """
    logger.info(f"complete_task called: user_id={user_id}, task_id={task_id}")

    try:
        # Validate input using Pydantic model
        input_data = CompleteTaskInput(
            user_id=user_id,
            task_id=task_id
        )

        # Get async database session
        async for session in get_async_session():
            # Verify user exists
            user = await session.get(User, input_data.user_id)
            if not user:
                logger.warning(f"User not found: {input_data.user_id}")
                raise ValueError(f"User {input_data.user_id} not found")

            # Retrieve task
            task = await session.get(Task, input_data.task_id)
            if not task:
                logger.warning(f"Task not found: {input_data.task_id}")
                raise TaskNotFoundError(f"Task {input_data.task_id} not found")

            # Verify ownership
            if task.user_id != input_data.user_id:
                logger.warning(f"Unauthorized access attempt: user {input_data.user_id} tried to complete task {input_data.task_id} owned by {task.user_id}")
                raise UnauthorizedTaskAccessError(f"Task {input_data.task_id} does not belong to user {input_data.user_id}")

            # Update task completion status
            task.completed = True
            task.updated_at = datetime.utcnow()

            # Persist changes
            await session.commit()
            await session.refresh(task)

            # Format response
            response = {
                "task_id": task.id,
                "status": "completed",
                "title": task.title,
                "description": task.description,
                "completed": task.completed,
                "created_at": task.created_at.isoformat() + "Z",
                "updated_at": task.updated_at.isoformat() + "Z"
            }

            logger.info(f"Task {task.id} marked as completed")
            return response

    except (ValueError, TaskNotFoundError, UnauthorizedTaskAccessError) as e:
        logger.error(f"Validation error in complete_task: {str(e)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in complete_task: {str(e)}")
        raise TaskToolError(f"Failed to complete task: {str(e)}")


@mcp.tool()
async def update_task(
    user_id: str,
    task_id: int,
    title: str = None,
    description: str = None
) -> dict:
    """Update a task's title and/or description.

    Args:
        user_id: The authenticated user's UUID
        task_id: The task ID to update
        title: New task title (optional, 1-255 characters)
        description: New task description (optional, max 10,000 characters)

    Returns:
        dict: Updated task details with status="updated"

    Raises:
        ValueError: If user not found or no fields provided
        TaskNotFoundError: If task not found
        UnauthorizedTaskAccessError: If user doesn't own the task
        ValidationError: If validation fails
    """
    logger.info(f"update_task called: user_id={user_id}, task_id={task_id}")

    try:
        # Validate input using Pydantic model
        input_data = UpdateTaskInput(
            user_id=user_id,
            task_id=task_id,
            title=title,
            description=description
        )

        # Get async database session
        async for session in get_async_session():
            # Verify user exists
            user = await session.get(User, input_data.user_id)
            if not user:
                logger.warning(f"User not found: {input_data.user_id}")
                raise ValueError(f"User {input_data.user_id} not found")

            # Retrieve task
            task = await session.get(Task, input_data.task_id)
            if not task:
                logger.warning(f"Task not found: {input_data.task_id}")
                raise TaskNotFoundError(f"Task {input_data.task_id} not found")

            # Verify ownership
            if task.user_id != input_data.user_id:
                logger.warning(f"Unauthorized access attempt: user {input_data.user_id} tried to update task {input_data.task_id} owned by {task.user_id}")
                raise UnauthorizedTaskAccessError(f"Task {input_data.task_id} does not belong to user {input_data.user_id}")

            # Update task fields (partial update)
            if input_data.title is not None:
                task.title = input_data.title
            if input_data.description is not None:
                task.description = input_data.description

            # Update timestamp
            task.updated_at = datetime.utcnow()

            # Persist changes
            await session.commit()
            await session.refresh(task)

            # Format response
            response = {
                "task_id": task.id,
                "status": "updated",
                "title": task.title,
                "description": task.description,
                "completed": task.completed,
                "created_at": task.created_at.isoformat() + "Z",
                "updated_at": task.updated_at.isoformat() + "Z"
            }

            logger.info(f"Task {task.id} updated successfully")
            return response

    except (ValueError, TaskNotFoundError, UnauthorizedTaskAccessError, ValidationError) as e:
        logger.error(f"Validation error in update_task: {str(e)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in update_task: {str(e)}")
        raise TaskToolError(f"Failed to update task: {str(e)}")


@mcp.tool()
async def delete_task(
    user_id: str,
    task_id: int
) -> dict:
    """Permanently delete a task.

    Args:
        user_id: The authenticated user's UUID
        task_id: The task ID to delete

    Returns:
        dict: Deleted task details with status="deleted"

    Raises:
        ValueError: If user not found
        TaskNotFoundError: If task not found
        UnauthorizedTaskAccessError: If user doesn't own the task
    """
    logger.info(f"delete_task called: user_id={user_id}, task_id={task_id}")

    try:
        # Validate input using Pydantic model
        input_data = DeleteTaskInput(
            user_id=user_id,
            task_id=task_id
        )

        # Get async database session
        async for session in get_async_session():
            # Verify user exists
            user = await session.get(User, input_data.user_id)
            if not user:
                logger.warning(f"User not found: {input_data.user_id}")
                raise ValueError(f"User {input_data.user_id} not found")

            # Retrieve task
            task = await session.get(Task, input_data.task_id)
            if not task:
                logger.warning(f"Task not found: {input_data.task_id}")
                raise TaskNotFoundError(f"Task {input_data.task_id} not found")

            # Verify ownership
            if task.user_id != input_data.user_id:
                logger.warning(f"Unauthorized access attempt: user {input_data.user_id} tried to delete task {input_data.task_id} owned by {task.user_id}")
                raise UnauthorizedTaskAccessError(f"Task {input_data.task_id} does not belong to user {input_data.user_id}")

            # Capture task details before deletion
            response = {
                "task_id": task.id,
                "status": "deleted",
                "title": task.title,
                "description": task.description,
                "completed": task.completed,
                "created_at": task.created_at.isoformat() + "Z",
                "updated_at": task.updated_at.isoformat() + "Z"
            }

            # Delete task from database
            await session.delete(task)
            await session.commit()

            logger.info(f"Task {task.id} deleted successfully")
            return response

    except (ValueError, TaskNotFoundError, UnauthorizedTaskAccessError) as e:
        logger.error(f"Validation error in delete_task: {str(e)}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in delete_task: {str(e)}")
        raise TaskToolError(f"Failed to delete task: {str(e)}")
