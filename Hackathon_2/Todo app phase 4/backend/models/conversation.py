from sqlmodel import SQLModel, Field, Relationship, Session, select
from typing import Optional, List
from datetime import datetime


class Conversation(SQLModel, table=True):
    """
    Represents a chat session between a user and the AI assistant.

    Relationships:
    - Belongs to one User (via user_id)
    - Has many Messages (via messages relationship)

    Data Isolation:
    - All queries MUST filter by authenticated user_id
    - Users can only access their own conversations
    """
    __tablename__ = "conversation"
    __table_args__ = {'extend_existing': True}

    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id", index=True)
    title: Optional[str] = Field(default=None, max_length=255)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow, sa_column_kwargs={"index": True})

    # Relationships
    messages: List["Message"] = Relationship(back_populates="conversation")

    @staticmethod
    def create_conversation(session: Session, user_id: str, title: Optional[str] = None) -> "Conversation":
        """
        Create a new conversation for a user.

        Args:
            session: Database session
            user_id: ID of the user creating the conversation (must exist in users table)
            title: Optional conversation title (max 255 characters)

        Returns:
            Created Conversation instance

        Raises:
            ValueError: If user_id is empty or title exceeds 255 characters
        """
        if not user_id or not user_id.strip():
            raise ValueError("user_id is required and cannot be empty")

        if title and len(title) > 255:
            raise ValueError("title cannot exceed 255 characters")

        conversation = Conversation(
            user_id=user_id,
            title=title,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )

        session.add(conversation)
        session.commit()
        session.refresh(conversation)

        return conversation

    @staticmethod
    def get_user_conversations(
        session: Session,
        user_id: str,
        limit: int = 50,
        offset: int = 0
    ) -> List["Conversation"]:
        """
        Get all conversations for a user, ordered by most recently updated.

        Args:
            session: Database session
            user_id: ID of the user
            limit: Maximum number of conversations to return (default 50)
            offset: Number of conversations to skip (default 0)

        Returns:
            List of Conversation instances ordered by updated_at DESC
        """
        statement = (
            select(Conversation)
            .where(Conversation.user_id == user_id)
            .order_by(Conversation.updated_at.desc())
            .limit(limit)
            .offset(offset)
        )

        results = session.exec(statement)
        return list(results.all())

    @staticmethod
    def get_conversation_by_id(session: Session, conversation_id: int, user_id: str) -> Optional["Conversation"]:
        """
        Get a single conversation by ID with user ownership validation.

        Args:
            session: Database session
            conversation_id: ID of the conversation
            user_id: ID of the user (for ownership validation)

        Returns:
            Conversation instance if found and owned by user, None otherwise
        """
        statement = select(Conversation).where(
            Conversation.id == conversation_id,
            Conversation.user_id == user_id
        )

        result = session.exec(statement)
        return result.first()

    def update_title(self, session: Session, new_title: str) -> None:
        """
        Update the conversation title and updated_at timestamp.

        Args:
            session: Database session
            new_title: New title for the conversation (max 255 characters)

        Raises:
            ValueError: If title exceeds 255 characters
        """
        if len(new_title) > 255:
            raise ValueError("title cannot exceed 255 characters")

        self.title = new_title
        self.updated_at = datetime.utcnow()

        session.add(self)
        session.commit()
        session.refresh(self)

    def touch(self, session: Session) -> None:
        """
        Update the updated_at timestamp (called when a message is added).

        Args:
            session: Database session
        """
        self.updated_at = datetime.utcnow()
        session.add(self)
        session.commit()
