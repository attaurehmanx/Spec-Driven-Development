# Research: Chat Conversation Persistence

**Feature**: 003-chat-database-schema
**Date**: 2026-01-29
**Purpose**: Resolve technical unknowns and establish implementation approach

## Research Questions

### Q1: SQLModel Relationship Patterns for One-to-Many

**Question**: What is the best practice for implementing one-to-many relationships in SQLModel (Conversation → Messages)?

**Decision**: Use SQLModel's `Relationship` field with `back_populates` for bidirectional navigation

**Rationale**:
- SQLModel provides native support for relationships through the `Relationship` type
- `back_populates` ensures bidirectional consistency between parent and child
- Follows SQLAlchemy best practices while maintaining Pydantic validation
- Enables lazy loading for performance optimization with large message lists

**Alternatives Considered**:
- Manual foreign key only (rejected: loses ORM navigation benefits)
- SQLAlchemy Core without ORM (rejected: loses type safety and validation)

**Implementation Pattern**:
```python
# Conversation model
messages: List["Message"] = Relationship(back_populates="conversation")

# Message model
conversation: "Conversation" = Relationship(back_populates="messages")
conversation_id: int = Field(foreign_key="conversations.id")
```

### Q2: User ID Type Consistency

**Question**: Should user_id be `str` or `int` across all models (Task, User, Conversation)?

**Decision**: Use `str` for user_id to match existing Phase 2 implementation and Better Auth requirements

**Rationale**:
- Better Auth (authentication system) uses string-based user identifiers
- Existing Task model likely uses `str` for user_id to match auth system
- Consistency across models prevents type conversion issues
- String IDs support various auth providers (UUID, email-based, etc.)

**Verification Required**: Check existing Task and User models to confirm user_id type

### Q3: Timestamp Handling

**Question**: How should we handle timezone-aware timestamps for created_at and updated_at?

**Decision**: Use `datetime` with UTC timezone, stored as `TIMESTAMP WITH TIME ZONE` in PostgreSQL

**Rationale**:
- Neon PostgreSQL supports timezone-aware timestamps natively
- UTC storage prevents timezone conversion issues
- SQLModel/SQLAlchemy handles timezone conversion automatically
- Follows industry best practice for distributed systems

**Implementation Pattern**:
```python
from datetime import datetime

created_at: datetime = Field(default_factory=datetime.utcnow)
```

**Note**: Existing backend uses `datetime.utcnow` (not `datetime.now(timezone.utc)`). We follow this pattern for consistency.

### Q4: Message Role Enumeration

**Question**: Should message roles (user, assistant, system) be stored as strings or enums?

**Decision**: Use Python `Enum` with string values, stored as VARCHAR in database

**Rationale**:
- Type safety at application level prevents invalid role values
- String storage in database maintains readability and flexibility
- Compatible with JSON serialization for API responses
- Follows OpenAI/LLM conversation format standards

**Implementation Pattern**:
```python
from enum import Enum

class MessageRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"

role: MessageRole = Field(...)
```

### Q5: Database Schema Creation Strategy

**Question**: How should we create the new tables (Conversation and Message)?

**Decision**: Use existing `SQLModel.metadata.create_all()` approach (no Alembic)

**Rationale**:
- Existing backend uses `database/migrations.py` with `SQLModel.metadata.create_all()`
- No Alembic configuration exists in the project
- Consistency with existing Phase 2 approach
- Simpler for development (automatic schema creation on startup)

**Implementation**: Add models to `backend/models/`, import in `__init__.py`, tables created automatically

## Technology Decisions

### Database Schema Design

**Indexes**:
- `conversations.user_id` - Required for efficient user-specific queries (FR-003)
- `conversations.updated_at` - Required for sorting by recency (FR-008)
- `messages.conversation_id` - Required for efficient message retrieval (FR-010)
- `messages.created_at` - Required for chronological ordering (FR-006)

**Constraints**:
- Foreign key: `conversations.user_id` → `users.id` (enforce user existence)
- Foreign key: `messages.conversation_id` → `conversations.id` (enforce conversation existence)
- NOT NULL: All required fields (id, user_id, created_at, role, content)
- CHECK constraint: `message.role` IN ('user', 'assistant', 'system')

**Field Sizes**:
- `conversation.title`: VARCHAR(255) - sufficient for descriptive titles
- `message.content`: TEXT - supports up to 10,000+ characters (SC-004)
- `message.role`: VARCHAR(20) - sufficient for enum values

### Performance Considerations

**Query Patterns**:
1. List user's conversations: `SELECT * FROM conversations WHERE user_id = ? ORDER BY updated_at DESC`
2. Get conversation messages: `SELECT * FROM messages WHERE conversation_id = ? ORDER BY created_at ASC`
3. Insert message: `INSERT INTO messages (...) VALUES (...)`

**Optimization Strategy**:
- Indexes on filter/sort columns (user_id, updated_at, conversation_id, created_at)
- Pagination for large conversation lists (LIMIT/OFFSET)
- Lazy loading for message relationships (don't load all messages with conversation list)

## Dependencies

**Existing**:
- SQLModel (already in Phase 2)
- Alembic (already in Phase 2)
- Neon PostgreSQL (already in Phase 2)
- FastAPI (already in Phase 2)

**New**: None - all required dependencies already present

## Assumptions Validated

1. ✅ Task and User models exist from Phase 2
2. ✅ Alembic is configured and working
3. ✅ Database connection is established
4. ✅ user_id type is consistent (to be verified in Phase 1)

## Next Steps

Proceed to Phase 1: Design & Contracts
- Create data-model.md with detailed Conversation and Message schemas
- Define SQLModel classes with proper relationships
- Generate Alembic migration script
- Create quickstart.md for testing database setup
