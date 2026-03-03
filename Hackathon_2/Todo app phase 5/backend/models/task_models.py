from sqlmodel import SQLModel, Field, Relationship, Column
from sqlalchemy import ARRAY, String, CheckConstraint
from typing import Optional, List
from datetime import datetime
from enum import Enum
import re


class PriorityLevel(str, Enum):
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class RecurringPattern(str, Enum):
    NONE = "none"
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"


class User(SQLModel, table=True):
    __table_args__ = {'extend_existing': True}
    id: str = Field(default_factory=lambda: str(__import__('uuid').uuid4()), primary_key=True)  # Auto-generate UUID
    email: str = Field(unique=True, index=True)
    hashed_password: str = Field(sa_column_kwargs={"name": "hashed_password"})  # Match DB column name
    name: str = Field(sa_column_kwargs={"name": "name"})  # Combined name field stored in DB
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    is_active: bool = Field(default=True)
    email_verified: bool = Field(default=False)
    email_verified_at: Optional[datetime] = Field(default=None)
    phone: Optional[str] = Field(default=None, sa_column_kwargs={"name": "phone"})
    address: Optional[str] = Field(default=None, sa_column_kwargs={"name": "address"})
    avatar: Optional[str] = Field(default=None, sa_column_kwargs={"name": "avatar"})

    # Relationship to tasks
    tasks: List["Task"] = Relationship(back_populates="owner")


class UserCreate(SQLModel):
    email: str
    password: str  # Raw password before hashing
    first_name: str
    last_name: str


class UserResponse(SQLModel):
    id: str
    email: str
    first_name: str
    last_name: str
    created_at: datetime
    is_active: bool = True
    email_verified: bool = False
    phone: Optional[str] = None
    address: Optional[str] = None
    avatar: Optional[str] = None

    class Config:
        from_attributes = True
        # Allow extra fields to ensure new fields are included
        extra = "allow"


class LoginRequest(SQLModel):
    email: str
    password: str


class TaskBase(SQLModel):
    title: str
    description: Optional[str] = None
    completed: bool = False
    priority: PriorityLevel = PriorityLevel.MEDIUM
    tags: List[str] = Field(default_factory=list, sa_column=Column(ARRAY(String), nullable=False, server_default="{}"))
    due_date: Optional[datetime] = None
    recurring: RecurringPattern = RecurringPattern.NONE


class Task(TaskBase, table=True):
    __table_args__ = (
        CheckConstraint("priority IN ('high', 'medium', 'low')", name="chk_priority"),
        CheckConstraint("recurring IN ('none', 'daily', 'weekly', 'monthly')", name="chk_recurring"),
        CheckConstraint("parent_task_id IS NULL OR parent_task_id != id", name="chk_no_self_parent"),
        {'extend_existing': True}
    )
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")  # UUID string to match user.id
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationship to owner
    owner: User = Relationship(back_populates="tasks")


class TaskCreate(TaskBase):
    @classmethod
    def validate_tags(cls, v):
        """Validate tag format: alphanumeric + hyphens, 1-50 chars, max 20 tags"""
        if v is None:
            return []
        if len(v) > 20:
            raise ValueError("Maximum 20 tags allowed per task")
        pattern = re.compile(r'^[a-zA-Z0-9-]{1,50}$')
        for tag in v:
            if not pattern.match(tag):
                raise ValueError(f"Invalid tag format: {tag}. Tags must be alphanumeric with hyphens, 1-50 characters")
        return v

    @classmethod
    def validate_due_date(cls, v):
        """Validate due date is not too far in past or future"""
        if v is not None:
            from datetime import timedelta
            now = datetime.utcnow()
            one_year_ago = now - timedelta(days=365)
            ten_years_future = now + timedelta(days=3650)

            if v < one_year_ago:
                raise ValueError("Due date cannot be more than 1 year in the past")
            if v > ten_years_future:
                raise ValueError("Due date cannot be more than 10 years in the future")
        return v


class TaskUpdate(SQLModel):
    title: Optional[str] = None
    description: Optional[str] = None
    completed: Optional[bool] = None
    priority: Optional[PriorityLevel] = None
    tags: Optional[List[str]] = None
    due_date: Optional[datetime] = None
    recurring: Optional[RecurringPattern] = None

    @classmethod
    def validate_tags(cls, v):
        """Validate tag format if tags are provided"""
        if v is not None:
            if len(v) > 20:
                raise ValueError("Maximum 20 tags allowed per task")
            pattern = re.compile(r'^[a-zA-Z0-9-]{1,50}$')
            for tag in v:
                if not pattern.match(tag):
                    raise ValueError(f"Invalid tag format: {tag}")
        return v


class TaskResponse(TaskBase):
    id: int
    user_id: str  # Changed from int to str to handle UUID
    parent_task_id: Optional[int] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

    @property
    def is_overdue(self) -> bool:
        """Computed property: task is overdue if due_date < now and not completed"""
        if self.due_date is None or self.completed:
            return False
        return datetime.utcnow() > self.due_date


class Token(SQLModel):
    access_token: str
    token_type: str


class RefreshTokenRequest(SQLModel):
    refresh_token: str


class Token(SQLModel):
    access_token: str
    refresh_token: Optional[str] = None  # Make refresh token optional for response compatibility
    token_type: str