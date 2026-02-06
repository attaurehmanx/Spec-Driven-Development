from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
from pydantic import BaseModel
from config.settings import get_settings
settings = get_settings()
import uuid


class TokenData(BaseModel):
    user_id: Optional[str] = None  # Changed from int to str to handle UUID
    email: Optional[str] = None
    sub: Optional[str] = None


class JWTHandler:
    def __init__(self, secret_key: str = settings.SECRET_KEY, algorithm: str = settings.ALGORITHM):
        self.secret_key = secret_key
        self.algorithm = algorithm

    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None):
        """Create an access token with the provided data"""
        to_encode = data.copy()

        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)
        return encoded_jwt

    def verify_token(self, token: str) -> Optional[TokenData]:
        """Verify a token and return the token data if valid"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            user_id: str = payload.get("sub")
            email: str = payload.get("email")

            if user_id is None:
                return None

            # Keep user_id as string for UUID handling
            token_data = TokenData(user_id=user_id, email=email, sub=user_id)
            return token_data
        except JWTError:
            return None

    def decode_token(self, token: str) -> Optional[dict]:
        """Decode a token without verification (useful for debugging)"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm], options={"verify_signature": False})
            return payload
        except JWTError:
            return None


# Global instance
jwt_handler = JWTHandler()