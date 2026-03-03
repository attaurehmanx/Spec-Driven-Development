from typing import Optional
from models.task_models import User
from utils.auth import TokenData
from utils.jwt import jwt_handler


class UserIdentityExtractor:
    """
    Utility class for extracting and validating user identity from JWT tokens
    """

    @staticmethod
    def extract_from_token(token: str) -> Optional[TokenData]:
        """
        Extract user identity from a JWT token
        """
        return jwt_handler.verify_token(token)

    @staticmethod
    def extract_user_id_from_token(token: str) -> Optional[str]:
        """
        Extract only the user ID from a JWT token
        """
        token_data = jwt_handler.verify_token(token)
        return token_data.user_id if token_data else None

    @staticmethod
    def extract_email_from_token(token: str) -> Optional[str]:
        """
        Extract only the email from a JWT token
        """
        token_data = jwt_handler.verify_token(token)
        return token_data.email if token_data else None

    @staticmethod
    def validate_user_identity_match(token_user_id: str, url_user_id: str) -> bool:
        """
        Validate that the user ID in the token matches the user ID in the URL parameter
        """
        # Compare the UUID strings directly since the database uses UUID strings
        return token_user_id == url_user_id

    @staticmethod
    def build_user_context_from_token(token: str) -> Optional[dict]:
        """
        Build a complete user context from a JWT token
        """
        token_data = jwt_handler.verify_token(token)
        if not token_data:
            return None

        return {
            "user_id": token_data.user_id,
            "email": token_data.email,
            "authenticated": True,
            "token_valid": True
        }

    @staticmethod
    def verify_token_and_extract_user_id(token: str, expected_user_id: str) -> tuple[bool, Optional[str]]:
        """
        Verify the token is valid and that the user ID matches the expected user ID
        Returns: (is_valid, user_id_if_valid)
        """
        token_data = jwt_handler.verify_token(token)

        if not token_data:
            return False, None

        if not UserIdentityExtractor.validate_user_identity_match(token_data.user_id, expected_user_id):
            return False, None

        return True, token_data.user_id


# Global instance for convenience
user_identity_extractor = UserIdentityExtractor()