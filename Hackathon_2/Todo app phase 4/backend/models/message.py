from sqlmodel import SQLModel, Field, Relationship, Session, select, Column
from typing import Optional, List
from datetime import datetime
from enum import Enum
import sqlalchemy as sa


class MessageRole(str, Enum):
    """
    Defines valid message sender roles in a conversation.

    Follows OpenAI/LLM conversation format standards.
    """
    USER = "user"          # Message from the human user
    ASSISTANT = "assistant"  # Message from the AI assistant
    SYSTEM = "system"       # System-generated message (e.g., notifications)


class Message(SQLModel, table=True):
    """
    Represents a single message within a conversation.

    Relationships:
    - Belongs to one Conversation (via conversation_id)

    Constraints:
    - Messages are immutable after creation
    - Content limited to 10,000 characters
    - Always ordered by created_at ASC within conversation
    """
    __tablename__ = "message"
    __table_args__ = {'extend_existing': True}

    id: Optional[int] = Field(default=None, primary_key=True)
    conversation_id: int = Field(
        sa_column=Column(
            sa.Integer,
            sa.ForeignKey("conversation.id", ondelete="CASCADE"),
            nullable=False,
            index=True
        )
    )
    role: str = Field(sa_column=Column(sa.Enum(MessageRole, values_callable=lambda x: [e.value for e in x])))
    content: str = Field(..., max_length=10000)
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column_kwargs={"index": True})

    # Relationships
    conversation: "Conversation" = Relationship(back_populates="messages")

    @staticmethod
    def add_message(
        session: Session,
        conversation_id: int,
        role: MessageRole,
        content: str
    ) -> "Message":
        """
        Add a new message to a conversation.

        Args:
            session: Database session
            conversation_id: ID of the conversation (must exist)
            role: Role of the message sender (USER, ASSISTANT, or SYSTEM)
            content: Message content (1-10,000 characters)

        Returns:
            Created Message instance

        Raises:
            ValueError: If conversation_id is invalid, content is empty/too long
        """
        # Import here to avoid circular dependency
        from .conversation import Conversation

        if not content or not content.strip():
            raise ValueError("content is required and cannot be empty")

        if len(content) > 10000:
            raise ValueError("content cannot exceed 10,000 characters")

        # Verify conversation exists
        conversation = session.get(Conversation, conversation_id)
        if not conversation:
            raise ValueError(f"conversation_id {conversation_id} does not exist")

        # Create message
        message = Message(
            conversation_id=conversation_id,
            role=role,
            content=content,
            created_at=datetime.utcnow()
        )

        session.add(message)

        # Update conversation's updated_at timestamp
        conversation.touch(session)

        session.commit()
        session.refresh(message)

        return message

    @staticmethod
    def get_conversation_messages(
        session: Session,
        conversation_id: int
    ) -> List["Message"]:
        """
        Get all messages for a conversation in chronological order.

        Args:
            session: Database session
            conversation_id: ID of the conversation

        Returns:
            List of Message instances ordered by created_at ASC
        """
        statement = (
            select(Message)
            .where(Message.conversation_id == conversation_id)
            .order_by(Message.created_at.asc())
        )

        results = session.exec(statement)
        return list(results.all())
