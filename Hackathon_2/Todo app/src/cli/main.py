"""
Console interface for the Todo Console Application
"""
import sys
import os
from typing import Optional

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.services.todo_service import TodoService


class TodoConsoleApp:
    """
    Console interface for the todo application.
    """
    def __init__(self):
        self.service = TodoService()

    def display_menu(self):
        """
        Display the main menu options.
        """
        print("\n" + "="*50)
        print("Welcome to the Todo Console Application!")
        print("="*50)
        print("1. Add Todo")
        print("2. View Todos")
        print("3. Update Todo")
        print("4. Delete Todo")
        print("5. Mark Complete/Incomplete")
        print("6. Exit")
        print("="*50)

    def get_user_choice(self) -> int:
        """
        Get and validate user menu choice.

        Returns:
            int: Valid menu choice (1-6)
        """
        while True:
            try:
                choice = int(input("Choose an option (1-6): "))
                if 1 <= choice <= 6:
                    return choice
                else:
                    print("Please enter a number between 1 and 6.")
            except ValueError:
                print("Please enter a valid number.")

    def add_todo(self):
        """
        Add a new todo.
        """
        try:
            title = input("Enter title: ").strip()
            if not title:
                print("Title cannot be empty.")
                return

            description = input("Enter description (optional): ").strip()
            if not description:
                description = None

            todo = self.service.add_todo(title, description)
            print(f"Todo added successfully with ID: {todo.id}")
        except ValueError as e:
            print(f"Error: {e}")

    def view_todos(self):
        """
        View all todos.
        """
        todos = self.service.get_all_todos()
        if not todos:
            print("No todos found.")
            return

        print("\nYour Todos:")
        print("-" * 80)
        for todo in todos:
            status_indicator = "✓" if todo.status == "Complete" else "○"
            description = todo.description if todo.description else ""
            print(f"{status_indicator} ID: {todo.id} | Title: {todo.title} | Description: {description} | Status: {todo.status}")
        print("-" * 80)

    def update_todo(self):
        """
        Update an existing todo.
        """
        try:
            todo_id = int(input("Enter todo ID to update: "))
        except ValueError:
            print("Please enter a valid number for the todo ID.")
            return

        # Check if todo exists
        existing_todo = self.service.get_todo(todo_id)
        if not existing_todo:
            print(f"Todo with ID {todo_id} not found.")
            return

        print(f"Current: ID: {existing_todo.id} | Title: {existing_todo.title} | Description: {existing_todo.description} | Status: {existing_todo.status}")

        new_title = input(f"Enter new title (current: '{existing_todo.title}', press Enter to keep current): ").strip()
        if not new_title:
            new_title = None  # Keep existing title

        new_description = input(f"Enter new description (current: '{existing_todo.description}', press Enter to keep current): ").strip()
        if new_description == "":
            new_description = None  # Keep existing description or set to None if was None

        try:
            updated_todo = self.service.update_todo(todo_id, new_title, new_description)
            if updated_todo:
                print("Todo updated successfully!")
            else:
                print("Failed to update todo. Please check the input.")
        except ValueError as e:
            print(f"Error: {e}")

    def delete_todo(self):
        """
        Delete a todo.
        """
        try:
            todo_id = int(input("Enter todo ID to delete: "))
        except ValueError:
            print("Please enter a valid number for the todo ID.")
            return

        # Check if todo exists
        existing_todo = self.service.get_todo(todo_id)
        if not existing_todo:
            print(f"Todo with ID {todo_id} not found.")
            return

        print(f"About to delete: ID: {existing_todo.id} | Title: {existing_todo.title}")
        confirm = input("Are you sure you want to delete this todo? (y/N): ").lower()

        if confirm in ['y', 'yes']:
            success = self.service.delete_todo(todo_id)
            if success:
                print("Todo deleted successfully!")
            else:
                print("Failed to delete todo.")
        else:
            print("Deletion cancelled.")

    def toggle_todo_status(self):
        """
        Toggle the completion status of a todo.
        """
        try:
            todo_id = int(input("Enter todo ID to toggle: "))
        except ValueError:
            print("Please enter a valid number for the todo ID.")
            return

        # Check if todo exists
        existing_todo = self.service.get_todo(todo_id)
        if not existing_todo:
            print(f"Todo with ID {todo_id} not found.")
            return

        updated_todo = self.service.toggle_todo_status(todo_id)
        if updated_todo:
            print(f"Todo status updated successfully! New status: {updated_todo.status}")
        else:
            print("Failed to update todo status.")

    def run(self):
        """
        Run the console application.
        """
        print("Starting Todo Console Application...")
        while True:
            self.display_menu()
            choice = self.get_user_choice()

            if choice == 1:
                self.add_todo()
            elif choice == 2:
                self.view_todos()
            elif choice == 3:
                self.update_todo()
            elif choice == 4:
                self.delete_todo()
            elif choice == 5:
                self.toggle_todo_status()
            elif choice == 6:
                print("Goodbye!")
                sys.exit(0)

            # Pause to let user see the result before showing the menu again
            input("\nPress Enter to continue...")


def main():
    """
    Main function to run the application.
    """
    app = TodoConsoleApp()
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nGoodbye!")
        sys.exit(0)


if __name__ == "__main__":
    main()