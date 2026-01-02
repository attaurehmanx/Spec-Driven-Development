# Quickstart: Todo In-Memory Python Console Application

## Getting Started

1. **Prerequisites**
   - Python 3.13 or higher
   - UV package manager

2. **Setup**
   ```bash
   # Clone the repository
   git clone <repository-url>

   # Navigate to project directory
   cd todoapp

   # Install dependencies with UV
   uv sync
   ```

3. **Run the Application**
   ```bash
   # Run the console application
   python src/cli/main.py
   ```

## Basic Usage

Once the application starts, you'll see a menu with the following options:

1. **Add Todo**: Create a new todo with title and optional description
2. **View Todos**: Display all todos with ID, title, description, and status
3. **Update Todo**: Modify the title or description of an existing todo
4. **Delete Todo**: Remove a todo (with confirmation)
5. **Mark Complete/Incomplete**: Toggle the completion status of a todo
6. **Exit**: Quit the application

## Example Workflow

```
Welcome to the Todo Console Application!

1. Add Todo
2. View Todos
3. Update Todo
4. Delete Todo
5. Mark Complete/Incomplete
6. Exit

Choose an option: 1
Enter title: Buy groceries
Enter description (optional):

Todo added successfully with ID: 1

Choose an option: 2
ID: 1 | Title: Buy groceries | Description:  | Status: Incomplete

Choose an option: 5
Enter todo ID to toggle: 1
Todo status updated successfully!

Choose an option: 2
ID: 1 | Title: Buy groceries | Description:  | Status: Complete

Choose an option: 6
Goodbye!
```

## Error Handling

- Invalid menu selections will prompt for valid input
- Non-existent todo IDs will show appropriate error messages
- Empty titles will be rejected when adding todos
- All errors provide user-friendly messages