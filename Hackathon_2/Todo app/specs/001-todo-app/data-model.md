# Data Model: Todo In-Memory Python Console Application

## Entity: Todo

### Fields
- **id**: Integer (required, unique incremental)
  - Auto-generated using a simple counter
  - Starts from 1 and increments for each new todo
  - Primary identifier for the todo item

- **title**: String (required)
  - User-provided title for the todo
  - Must not be empty or null
  - Maximum length: reasonable console display limit (e.g., 200 characters)

- **description**: String (optional)
  - User-provided description for the todo
  - Can be empty or null
  - Maximum length: reasonable console display limit (e.g., 500 characters)

- **status**: String (required)
  - Values: "Complete" or "Incomplete"
  - Default value: "Incomplete"
  - Represents the completion status of the todo

### Validation Rules
- **Title required**: Todo must have a non-empty title
- **Unique ID**: Each todo must have a unique incremental ID
- **Status values**: Status can only be "Complete" or "Incomplete"

### State Transitions
- **Incomplete → Complete**: When user marks todo as complete
- **Complete → Incomplete**: When user marks todo as incomplete

### Relationships
- None (Todo is a standalone entity in this application)

## Storage Model: TodoList

### Fields
- **todos**: List of Todo objects
  - In-memory collection of all todos
  - Provides methods for CRUD operations
  - Maintains the incremental ID sequence

### Operations
- **Add Todo**: Adds a new todo to the list with next available ID
- **Get Todo by ID**: Retrieves a todo by its unique ID
- **Update Todo**: Modifies an existing todo's properties
- **Delete Todo**: Removes a todo from the list
- **Toggle Status**: Changes the completion status of a todo
- **Get All Todos**: Returns all todos in the list