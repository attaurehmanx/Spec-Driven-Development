from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime


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


class Task(TaskBase, table=True):
    __table_args__ = {'extend_existing': True}
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")  # UUID string to match user.id
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationship to owner
    owner: User = Relationship(back_populates="tasks")


class TaskCreate(TaskBase):
    pass


class TaskUpdate(SQLModel):
    title: Optional[str] = None
    description: Optional[str] = None
    completed: Optional[bool] = None


class TaskResponse(TaskBase):
    id: int
    user_id: str  # Changed from int to str to handle UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class Token(SQLModel):
    access_token: str
    token_type: str


class RefreshTokenRequest(SQLModel):
    refresh_token: str


class Token(SQLModel):
    access_token: str
    refresh_token: Optional[str] = None  # Make refresh token optional for response compatibility
    token_type: str