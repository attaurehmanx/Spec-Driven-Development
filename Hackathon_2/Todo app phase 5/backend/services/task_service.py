from sqlmodel import Session, select, col
from typing import List, Optional
from models.task_models import Task, PriorityLevel
from datetime import datetime


class TaskService:
    """Service layer for task operations including search, filter, and sort"""

    @staticmethod
    def filter_tasks(
        session: Session,
        user_id: str,
        priority: Optional[PriorityLevel] = None,
        tags: Optional[List[str]] = None,
        status: Optional[str] = None,
        search: Optional[str] = None
    ) -> List[Task]:
        """
        Filter tasks by various criteria

        Args:
            session: Database session
            user_id: User ID to filter by
            priority: Priority level (high, medium, low)
            tags: List of tags to filter by (OR logic - matches any tag)
            status: Completion status (done, not_done)
            search: Search term for title and description (case-insensitive)

        Returns:
            List of filtered tasks
        """
        # Start with base query for user's tasks
        statement = select(Task).where(Task.user_id == user_id)

        # Apply priority filter
        if priority:
            statement = statement.where(Task.priority == priority)

        # Apply status filter
        if status:
            if status == "done":
                statement = statement.where(Task.completed == True)
            elif status == "not_done":
                statement = statement.where(Task.completed == False)

        # Apply tags filter (PostgreSQL array overlap operator)
        if tags and len(tags) > 0:
            # Use PostgreSQL array overlap operator &&
            statement = statement.where(col(Task.tags).op("&&")(tags))

        # Apply search filter (case-insensitive ILIKE on title and description)
        if search:
            search_pattern = f"%{search}%"
            statement = statement.where(
                (Task.title.ilike(search_pattern)) |
                (Task.description.ilike(search_pattern))
            )

        # Execute query
        tasks = session.exec(statement).all()
        return tasks

    @staticmethod
    def sort_tasks(tasks: List[Task], sort_by: Optional[str] = None) -> List[Task]:
        """
        Sort tasks by specified criteria

        Args:
            tasks: List of tasks to sort
            sort_by: Sort criteria (priority, due_date, created_at, title)

        Returns:
            Sorted list of tasks
        """
        if not sort_by:
            # Default: newest first
            return sorted(tasks, key=lambda t: t.created_at, reverse=True)

        if sort_by == "priority":
            # Sort by priority: high (1) -> medium (2) -> low (3)
            priority_order = {"high": 1, "medium": 2, "low": 3}
            return sorted(tasks, key=lambda t: priority_order.get(t.priority.value, 4))

        elif sort_by == "due_date":
            # Sort by due date: overdue first, then nearest due date first, then no due date
            def due_date_key(task):
                if task.due_date is None:
                    return (2, datetime.max)  # No due date goes last
                elif task.due_date < datetime.utcnow() and not task.completed:
                    return (0, task.due_date)  # Overdue tasks first
                else:
                    return (1, task.due_date)  # Future tasks by due date

            return sorted(tasks, key=due_date_key)

        elif sort_by == "created_at":
            # Sort by creation date: newest first
            return sorted(tasks, key=lambda t: t.created_at, reverse=True)

        elif sort_by == "title":
            # Sort alphabetically by title (A-Z)
            return sorted(tasks, key=lambda t: t.title.lower())

        else:
            # Unknown sort criteria, return as-is
            return tasks

    @staticmethod
    def search_tasks(
        session: Session,
        user_id: str,
        query: str
    ) -> List[Task]:
        """
        Search tasks by title or description

        Args:
            session: Database session
            user_id: User ID to filter by
            query: Search query string

        Returns:
            List of matching tasks
        """
        search_pattern = f"%{query}%"
        statement = select(Task).where(
            Task.user_id == user_id,
            (Task.title.ilike(search_pattern)) |
            (Task.description.ilike(search_pattern))
        )

        tasks = session.exec(statement).all()
        return tasks

    @staticmethod
    def get_overdue_tasks(
        session: Session,
        user_id: str
    ) -> List[Task]:
        """
        Get all overdue tasks for a user

        Args:
            session: Database session
            user_id: User ID to filter by

        Returns:
            List of overdue tasks (due_date < NOW() AND completed = false)
        """
        now = datetime.utcnow()
        statement = select(Task).where(
            Task.user_id == user_id,
            Task.due_date < now,
            Task.completed == False
        )

        tasks = session.exec(statement).all()
        return tasks
