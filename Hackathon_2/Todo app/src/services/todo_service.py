"""
Todo service for the Todo Console Application
"""
from typing import List, Optional
from src.models.todo import Todo
from src.lib.storage import TodoStorage


class TodoService:
    """
    Service layer for todo operations.
    """
    def __init__(self):
        self.storage = TodoStorage()

    def add_todo(self, title: str, description: Optional[str] = None) -> Todo:
        """
        Add a new todo.

        Args:
            title: Title of the todo
            description: Optional description of the todo

        Returns:
            Todo: The created todo with assigned ID

        Raises:
            ValueError: If title is empty
        """
        if not title or not title.strip():
            raise ValueError("Title cannot be empty")

        return self.storage.add_todo(title, description)

    def get_todo(self, todo_id: int) -> Optional[Todo]:
        """
        Get a todo by its ID.

        Args:
            todo_id: ID of the todo to retrieve

        Returns:
            Todo: The todo if found, None otherwise
        """
        return self.storage.get_todo(todo_id)

    def get_all_todos(self) -> List[Todo]:
        """
        Get all todos.

        Returns:
            List[Todo]: List of all todos
        """
        return self.storage.get_all_todos()

    def update_todo(self, todo_id: int, title: Optional[str] = None,
                    description: Optional[str] = None) -> Optional[Todo]:
        """
        Update an existing todo.

        Args:
            todo_id: ID of the todo to update
            title: New title (optional)
            description: New description (optional)

        Returns:
            Todo: Updated todo if successful, None if todo doesn't exist or validation fails
        """
        if title is not None and (not title or not title.strip()):
            raise ValueError("Title cannot be empty")

        return self.storage.update_todo(todo_id, title, description)

    def delete_todo(self, todo_id: int) -> bool:
        """
        Delete a todo by its ID.

        Args:
            todo_id: ID of the todo to delete

        Returns:
            bool: True if deletion was successful, False if todo doesn't exist
        """
        return self.storage.delete_todo(todo_id)

    def toggle_todo_status(self, todo_id: int) -> Optional[Todo]:
        """
        Toggle the completion status of a todo.

        Args:
            todo_id: ID of the todo to toggle

        Returns:
            Todo: Updated todo if successful, None if todo doesn't exist
        """
        return self.storage.toggle_todo_status(todo_id)

    def get_next_id(self) -> int:
        """
        Get the next available ID.

        Returns:
            int: The next available ID
        """
        return self.storage.get_next_id()