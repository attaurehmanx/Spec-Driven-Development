"""
Todo data model for the Todo Console Application
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class Todo:
    """
    Represents a todo item with ID, title, description, and status.
    """
    id: int
    title: str
    description: Optional[str] = None
    status: str = "Incomplete"

    def __post_init__(self):
        """
        Validate the todo after initialization.
        """
        if not self.title or not self.title.strip():
            raise ValueError("Title cannot be empty")

        if self.status not in ["Complete", "Incomplete"]:
            raise ValueError("Status must be either 'Complete' or 'Incomplete'")

    def validate(self) -> bool:
        """
        Validate the todo object.

        Returns:
            bool: True if the todo is valid, False otherwise
        """
        try:
            self.__post_init__()
            return True
        except ValueError:
            return False

    def to_dict(self) -> dict:
        """
        Convert the todo object to a dictionary.

        Returns:
            dict: Dictionary representation of the todo
        """
        return {
            "id": self.id,
            "title": self.title,
            "description": self.description,
            "status": self.status
        }

    @classmethod
    def from_dict(cls, data: dict):
        """
        Create a Todo object from a dictionary.

        Args:
            data: Dictionary containing todo data

        Returns:
            Todo: Todo object created from the dictionary
        """
        return cls(
            id=data["id"],
            title=data["title"],
            description=data.get("description"),
            status=data.get("status", "Incomplete")
        )