"""
Recurring Task Service

Handles the creation of recurring task instances when a recurring task is completed.
Implements business logic for calculating next due dates based on recurrence patterns.
"""

from datetime import datetime, timedelta
from typing import Optional
from calendar import monthrange

from models.task_models import Task, TaskCreate, RecurringPattern


class RecurringService:
    """Service for managing recurring task instances."""

    @staticmethod
    def calculate_next_due_date(
        current_due_date: Optional[datetime],
        recurring_pattern: RecurringPattern
    ) -> Optional[datetime]:
        """
        Calculate the next due date based on the recurring pattern.

        Args:
            current_due_date: The current task's due date
            recurring_pattern: The recurrence pattern (DAILY, WEEKLY, MONTHLY)

        Returns:
            The calculated next due date, or None if no due date or pattern is NONE

        Edge Cases:
            - If current_due_date is None, returns None
            - For MONTHLY: Handles month-end dates (Jan 31 → Feb 28/29)
            - For MONTHLY: Preserves day of month when possible (Jan 15 → Feb 15)
        """
        if not current_due_date or recurring_pattern == RecurringPattern.NONE:
            return None

        if recurring_pattern == RecurringPattern.DAILY:
            # Add 1 day
            return current_due_date + timedelta(days=1)

        elif recurring_pattern == RecurringPattern.WEEKLY:
            # Add 7 days
            return current_due_date + timedelta(weeks=1)

        elif recurring_pattern == RecurringPattern.MONTHLY:
            # Add 1 month, handling month-end edge cases
            current_year = current_due_date.year
            current_month = current_due_date.month
            current_day = current_due_date.day

            # Calculate next month and year
            next_month = current_month + 1
            next_year = current_year

            if next_month > 12:
                next_month = 1
                next_year += 1

            # Get the last day of the next month
            last_day_of_next_month = monthrange(next_year, next_month)[1]

            # If current day is greater than last day of next month,
            # use the last day of next month (e.g., Jan 31 → Feb 28)
            next_day = min(current_day, last_day_of_next_month)

            # Create the next due date with the same time
            return current_due_date.replace(
                year=next_year,
                month=next_month,
                day=next_day
            )

        return None

    @staticmethod
    def create_next_instance(
        original_task: Task,
        user_id: str
    ) -> TaskCreate:
        """
        Create the next instance of a recurring task.

        Args:
            original_task: The completed recurring task
            user_id: The user ID who owns the task

        Returns:
            TaskCreate object for the new recurring instance

        Inheritance Rules:
            - Inherits: title, description, priority, tags, recurring pattern
            - New values: completed=False, new due_date, parent_task_id=original_task.id
            - Does NOT inherit: id, created_at, updated_at, completed status
        """
        # Calculate the next due date
        next_due_date = RecurringService.calculate_next_due_date(
            original_task.due_date,
            original_task.recurring
        )

        # Create the new task instance
        new_task = TaskCreate(
            title=original_task.title,
            description=original_task.description,
            completed=False,
            priority=original_task.priority,
            tags=original_task.tags or [],
            due_date=next_due_date,
            recurring=original_task.recurring,
            parent_task_id=original_task.id
        )

        return new_task

    @staticmethod
    def should_create_next_instance(task: Task) -> bool:
        """
        Determine if a next instance should be created for this task.

        Args:
            task: The task being completed

        Returns:
            True if a next instance should be created, False otherwise

        Rules:
            - Task must have a recurring pattern other than NONE
            - Task must be marked as completed
        """
        return (
            task.recurring != RecurringPattern.NONE and
            task.completed
        )
