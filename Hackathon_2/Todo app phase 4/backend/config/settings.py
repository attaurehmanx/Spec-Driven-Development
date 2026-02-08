from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Database Configuration
    DATABASE_URL: str

    # Authentication Configuration
    SECRET_KEY: str
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 5

    # Gemini API Configuration (optional - used by agent service)
    GEMINI_API_KEY: Optional[str] = None
    GEMINI_MODEL: Optional[str] = "gemini-2.5-flash"
    GEMINI_BASE_URL: Optional[str] = "https://generativelanguage.googleapis.com/v1beta/openai/"

    # Agent Configuration (optional - used by agent service)
    AGENT_MAX_ITERATIONS: Optional[int] = 15
    AGENT_TEMPERATURE: Optional[float] = 1.0
    AGENT_MAX_TOKENS: Optional[int] = 1000
    AGENT_TIMEOUT: Optional[int] = 30

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


def get_settings() -> Settings:
    return Settings()