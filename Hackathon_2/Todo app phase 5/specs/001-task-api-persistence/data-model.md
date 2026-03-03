# Data Model: Task API & Persistence Layer

## Task Entity

**Task**: A user's to-do item with properties including ID, title, description, completion status, creation timestamp, and user association
- **id**: Unique identifier (UUID or integer auto-increment)
- **title**: Task title (string, required, max length 255)
- **description**: Task description (string, optional, max length 1000)
- **completed**: Completion status (boolean, default false)
- **created_at**: Creation timestamp (datetime, auto-generated)
- **updated_at**: Last update timestamp (datetime, auto-updated)
- **user_id**: Foreign key linking to the owning user (string/UUID, required, indexed)

## Relationship Patterns

**User-Task Association**:
- One user can own many tasks (1:N relationship)
- Task.user_id references User.id
- Cascading deletes are not allowed to preserve data integrity
- Index on user_id for efficient querying

## Validation Rules

**Task Creation**:
- Title is required and must be between 1-255 characters
- Description is optional and must be under 1000 characters if provided
- Completed status defaults to false
- User_id must correspond to an existing authenticated user

**Task Updates**:
- User can only update tasks they own (validated against JWT claims)
- Title and description validation rules apply
- System automatically updates updated_at timestamp

## State Transitions

**Task Lifecycle**:
1. **Created**: New task with completed=False
2. **Updated**: Task details modified by owner
3. **Completed**: Toggle completion status to completed=True
4. **Reopened**: Toggle completion status back to completed=False
5. **Deleted**: Task permanently removed by owner

## Query Patterns

**Efficient Access Patterns**:
- Retrieve all tasks for a specific user_id (primary use case)
- Retrieve single task by ID with user ownership validation
- Filter tasks by completion status for a specific user
- Count tasks for a specific user