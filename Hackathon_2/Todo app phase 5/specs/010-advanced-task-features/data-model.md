# Data Model: Advanced Task Management Features

**Feature**: 010-advanced-task-features
**Date**: 2026-02-15
**Purpose**: Database schema and entity definitions

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                           Task                              │
├─────────────────────────────────────────────────────────────┤
│ id: UUID (PK)                                               │
│ user_id: UUID (FK → users.id)                               │
│ title: String                                               │
│ description: String (nullable)                              │
│ completed: Boolean                                          │
│ priority: Enum('high', 'medium', 'low') DEFAULT 'medium'    │
│ tags: Array<String> DEFAULT []                              │
│ due_date: Timestamp (nullable)                              │
│ recurring: Enum('none', 'daily', 'weekly', 'monthly')       │
│             DEFAULT 'none'                                  │
│ parent_task_id: UUID (nullable, FK → tasks.id)              │
│ created_at: Timestamp                                       │
│ updated_at: Timestamp                                       │
└─────────────────────────────────────────────────────────────┘
         │
         │ (self-referential)
         │ parent_task_id → id
         │ (for recurring task lineage)
         └─────────────────┐
                           │
                           ▼
```

## Schema Definition

### Task Table (Extended)

**Table Name**: `tasks`

**Columns**:

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY | Unique task identifier |
| user_id | UUID | NOT NULL, FK(users.id) | Owner of the task |
| title | VARCHAR(255) | NOT NULL | Task title |
| description | TEXT | NULL | Detailed task description |
| completed | BOOLEAN | NOT NULL, DEFAULT false | Completion status |
| priority | VARCHAR(10) | NOT NULL, DEFAULT 'medium', CHECK(priority IN ('high', 'medium', 'low')) | Task priority level |
| tags | TEXT[] | NOT NULL, DEFAULT '{}' | Array of tag strings |
| due_date | TIMESTAMP | NULL | Task deadline (date + time) |
| recurring | VARCHAR(10) | NOT NULL, DEFAULT 'none', CHECK(recurring IN ('none', 'daily', 'weekly', 'monthly')) | Recurrence pattern |
| parent_task_id | UUID | NULL, FK(tasks.id) | Reference to parent task (for recurring instances) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |

**Indexes**:

```sql
-- Primary key index (automatic)
CREATE UNIQUE INDEX pk_tasks ON tasks(id);

-- Foreign key indexes
CREATE INDEX idx_tasks_user_id ON tasks(user_id);
CREATE INDEX idx_tasks_parent_task_id ON tasks(parent_task_id) WHERE parent_task_id IS NOT NULL;

-- Query optimization indexes
CREATE INDEX idx_tasks_priority ON tasks(priority);
CREATE INDEX idx_tasks_tags ON tasks USING GIN(tags);
CREATE INDEX idx_tasks_due_date ON tasks(due_date) WHERE due_date IS NOT NULL;
CREATE INDEX idx_tasks_user_priority ON tasks(user_id, priority);
CREATE INDEX idx_tasks_user_completed ON tasks(user_id, completed);

-- Composite index for common query patterns
CREATE INDEX idx_tasks_user_due_completed ON tasks(user_id, due_date, completed) WHERE due_date IS NOT NULL;
```

**Constraints**:

```sql
-- Foreign key constraints
ALTER TABLE tasks ADD CONSTRAINT fk_tasks_user_id
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE;

ALTER TABLE tasks ADD CONSTRAINT fk_tasks_parent_task_id
    FOREIGN KEY (parent_task_id) REFERENCES tasks(id) ON DELETE SET NULL;

-- Check constraints
ALTER TABLE tasks ADD CONSTRAINT chk_priority
    CHECK (priority IN ('high', 'medium', 'low'));

ALTER TABLE tasks ADD CONSTRAINT chk_recurring
    CHECK (recurring IN ('none', 'daily', 'weekly', 'monthly'));

-- Prevent self-referential loops
ALTER TABLE tasks ADD CONSTRAINT chk_no_self_parent
    CHECK (parent_task_id IS NULL OR parent_task_id != id);
```

## SQLModel Definition

```python
from sqlmodel import SQLModel, Field, Column
from sqlalchemy import ARRAY, String, CheckConstraint
from typing import Optional, List
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum

class PriorityLevel(str, Enum):
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"

class RecurringPattern(str, Enum):
    NONE = "none"
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"

class Task(SQLModel, table=True):
    __tablename__ = "tasks"

    # Primary key
    id: UUID = Field(default_factory=uuid4, primary_key=True)

    # Foreign keys
    user_id: UUID = Field(foreign_key="users.id", nullable=False, index=True)
    parent_task_id: Optional[UUID] = Field(
        default=None,
        foreign_key="tasks.id",
        nullable=True
    )

    # Core fields
    title: str = Field(max_length=255, nullable=False)
    description: Optional[str] = Field(default=None, nullable=True)
    completed: bool = Field(default=False, nullable=False)

    # New fields (Phase 1)
    priority: PriorityLevel = Field(default=PriorityLevel.MEDIUM, nullable=False)
    tags: List[str] = Field(
        default_factory=list,
        sa_column=Column(ARRAY(String), nullable=False, server_default="{}")
    )
    due_date: Optional[datetime] = Field(default=None, nullable=True)
    recurring: RecurringPattern = Field(default=RecurringPattern.NONE, nullable=False)

    # Timestamps
    created_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)
    updated_at: datetime = Field(default_factory=datetime.utcnow, nullable=False)

    # Table constraints
    __table_args__ = (
        CheckConstraint("parent_task_id IS NULL OR parent_task_id != id", name="chk_no_self_parent"),
    )
```

## Pydantic Schemas

### Request Schemas

```python
from pydantic import BaseModel, Field, validator
from typing import Optional, List
from datetime import datetime
from uuid import UUID

class TaskCreate(BaseModel):
    title: str = Field(min_length=1, max_length=255)
    description: Optional[str] = None
    priority: PriorityLevel = PriorityLevel.MEDIUM
    tags: List[str] = Field(default_factory=list, max_items=20)
    due_date: Optional[datetime] = None
    recurring: RecurringPattern = RecurringPattern.NONE

    @validator('tags')
    def validate_tags(cls, v):
        # Validate each tag format
        import re
        pattern = re.compile(r'^[a-zA-Z0-9-]{1,50}$')
        for tag in v:
            if not pattern.match(tag):
                raise ValueError(f"Invalid tag format: {tag}")
        return v

    @validator('due_date')
    def validate_due_date(cls, v):
        if v is not None:
            # Validate due date is not too far in past or future
            from datetime import timedelta
            now = datetime.utcnow()
            one_year_ago = now - timedelta(days=365)
            ten_years_future = now + timedelta(days=3650)

            if v < one_year_ago:
                raise ValueError("Due date cannot be more than 1 year in the past")
            if v > ten_years_future:
                raise ValueError("Due date cannot be more than 10 years in the future")
        return v

class TaskUpdate(BaseModel):
    title: Optional[str] = Field(None, min_length=1, max_length=255)
    description: Optional[str] = None
    completed: Optional[bool] = None
    priority: Optional[PriorityLevel] = None
    tags: Optional[List[str]] = Field(None, max_items=20)
    due_date: Optional[datetime] = None
    recurring: Optional[RecurringPattern] = None

    @validator('tags')
    def validate_tags(cls, v):
        if v is not None:
            import re
            pattern = re.compile(r'^[a-zA-Z0-9-]{1,50}$')
            for tag in v:
                if not pattern.match(tag):
                    raise ValueError(f"Invalid tag format: {tag}")
        return v
```

### Response Schemas

```python
class TaskResponse(BaseModel):
    id: UUID
    user_id: UUID
    title: str
    description: Optional[str]
    completed: bool
    priority: PriorityLevel
    tags: List[str]
    due_date: Optional[datetime]
    recurring: RecurringPattern
    parent_task_id: Optional[UUID]
    created_at: datetime
    updated_at: datetime
    is_overdue: bool  # Computed field

    class Config:
        orm_mode = True

    @property
    def is_overdue(self) -> bool:
        if self.due_date is None or self.completed:
            return False
        return datetime.utcnow() > self.due_date
```

## Migration Script

**File**: `backend/alembic/versions/010_add_advanced_task_features.py`

```python
"""Add advanced task features

Revision ID: 010_advanced_features
Revises: 009_containerization
Create Date: 2026-02-15

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '010_advanced_features'
down_revision = '009_containerization'
branch_labels = None
depends_on = None

def upgrade():
    # Add new columns
    op.add_column('tasks', sa.Column('priority', sa.String(10), nullable=False, server_default='medium'))
    op.add_column('tasks', sa.Column('tags', postgresql.ARRAY(sa.String()), nullable=False, server_default='{}'))
    op.add_column('tasks', sa.Column('due_date', sa.TIMESTAMP(), nullable=True))
    op.add_column('tasks', sa.Column('recurring', sa.String(10), nullable=False, server_default='none'))
    op.add_column('tasks', sa.Column('parent_task_id', sa.UUID(), nullable=True))

    # Add check constraints
    op.create_check_constraint(
        'chk_priority',
        'tasks',
        "priority IN ('high', 'medium', 'low')"
    )
    op.create_check_constraint(
        'chk_recurring',
        'tasks',
        "recurring IN ('none', 'daily', 'weekly', 'monthly')"
    )
    op.create_check_constraint(
        'chk_no_self_parent',
        'tasks',
        "parent_task_id IS NULL OR parent_task_id != id"
    )

    # Add foreign key constraint
    op.create_foreign_key(
        'fk_tasks_parent_task_id',
        'tasks', 'tasks',
        ['parent_task_id'], ['id'],
        ondelete='SET NULL'
    )

    # Create indexes
    op.create_index('idx_tasks_priority', 'tasks', ['priority'])
    op.create_index('idx_tasks_tags', 'tasks', ['tags'], postgresql_using='gin')
    op.create_index('idx_tasks_due_date', 'tasks', ['due_date'], postgresql_where='due_date IS NOT NULL')
    op.create_index('idx_tasks_user_priority', 'tasks', ['user_id', 'priority'])
    op.create_index('idx_tasks_user_completed', 'tasks', ['user_id', 'completed'])
    op.create_index('idx_tasks_parent_task_id', 'tasks', ['parent_task_id'], postgresql_where='parent_task_id IS NOT NULL')
    op.create_index('idx_tasks_user_due_completed', 'tasks', ['user_id', 'due_date', 'completed'], postgresql_where='due_date IS NOT NULL')

def downgrade():
    # Drop indexes
    op.drop_index('idx_tasks_user_due_completed', 'tasks')
    op.drop_index('idx_tasks_parent_task_id', 'tasks')
    op.drop_index('idx_tasks_user_completed', 'tasks')
    op.drop_index('idx_tasks_user_priority', 'tasks')
    op.drop_index('idx_tasks_due_date', 'tasks')
    op.drop_index('idx_tasks_tags', 'tasks')
    op.drop_index('idx_tasks_priority', 'tasks')

    # Drop foreign key
    op.drop_constraint('fk_tasks_parent_task_id', 'tasks', type_='foreignkey')

    # Drop check constraints
    op.drop_constraint('chk_no_self_parent', 'tasks', type_='check')
    op.drop_constraint('chk_recurring', 'tasks', type_='check')
    op.drop_constraint('chk_priority', 'tasks', type_='check')

    # Drop columns
    op.drop_column('tasks', 'parent_task_id')
    op.drop_column('tasks', 'recurring')
    op.drop_column('tasks', 'due_date')
    op.drop_column('tasks', 'tags')
    op.drop_column('tasks', 'priority')
```

## Data Validation Rules

### Priority
- Must be one of: 'high', 'medium', 'low'
- Defaults to 'medium' if not specified
- Cannot be null

### Tags
- Array of strings
- Each tag must match pattern: `^[a-zA-Z0-9-]{1,50}$`
- Maximum 20 tags per task
- Defaults to empty array
- Cannot be null (empty array is valid)

### Due Date
- Optional (nullable)
- Must be valid ISO 8601 timestamp
- Cannot be more than 1 year in the past
- Cannot be more than 10 years in the future
- Includes both date and time components

### Recurring
- Must be one of: 'none', 'daily', 'weekly', 'monthly'
- Defaults to 'none' if not specified
- Cannot be null

### Parent Task ID
- Optional (nullable)
- Must reference existing task ID
- Cannot reference self (enforced by check constraint)
- Used to track recurring task lineage

## Query Patterns

### Common Queries

**Get all tasks for user with filters**:
```sql
SELECT * FROM tasks
WHERE user_id = $1
  AND ($2::text IS NULL OR priority = $2)
  AND ($3::text[] IS NULL OR tags && $3)
  AND ($4::boolean IS NULL OR completed = $4)
ORDER BY
  CASE WHEN $5 = 'priority' THEN
    CASE priority
      WHEN 'high' THEN 1
      WHEN 'medium' THEN 2
      WHEN 'low' THEN 3
    END
  END,
  CASE WHEN $5 = 'due_date' THEN due_date END,
  CASE WHEN $5 = 'created_at' THEN created_at END DESC,
  CASE WHEN $5 = 'title' THEN title END
LIMIT 1000;
```

**Search tasks by title or description**:
```sql
SELECT * FROM tasks
WHERE user_id = $1
  AND (title ILIKE $2 OR description ILIKE $2)
ORDER BY created_at DESC
LIMIT 100;
```

**Get overdue tasks**:
```sql
SELECT * FROM tasks
WHERE user_id = $1
  AND due_date < NOW()
  AND completed = false
ORDER BY due_date ASC;
```

**Get tasks with upcoming due dates (for reminders)**:
```sql
SELECT * FROM tasks
WHERE due_date BETWEEN NOW() AND NOW() + INTERVAL '1 hour'
  AND completed = false
ORDER BY due_date ASC;
```

## Storage Estimates

**Per Task**:
- Fixed fields: ~200 bytes
- Variable fields (title, description): ~500 bytes average
- Tags array: ~100 bytes average (5 tags × 20 chars)
- Total: ~800 bytes per task

**Per User** (500 tasks):
- 500 tasks × 800 bytes = 400 KB

**System** (10,000 users):
- 10,000 users × 400 KB = 4 GB
- Indexes: ~1.5 GB
- Total: ~5.5 GB

**Conclusion**: Storage requirements are reasonable for expected scale.
