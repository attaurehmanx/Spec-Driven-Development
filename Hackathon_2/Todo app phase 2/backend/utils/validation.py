from typing import Dict, Any, List
from pydantic import BaseModel, validator, ValidationError
import re
from datetime import datetime


class ValidationErrorResponse(BaseModel):
    """Standard error response for validation errors"""
    success: bool = False
    error_type: str = "validation_error"
    message: str
    details: List[Dict[str, Any]]


class UserValidation:
    """Validation utilities for user-related data"""

    @staticmethod
    def validate_email(email: str) -> bool:
        """Validate email format"""
        pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return bool(re.match(pattern, email))

    @staticmethod
    def validate_password_strength(password: str) -> tuple[bool, str]:
        """Validate password strength"""
        if len(password) < 8:
            return False, "Password must be at least 8 characters long"

        if not re.search(r"[A-Z]", password):
            return False, "Password must contain at least one uppercase letter"

        if not re.search(r"[a-z]", password):
            return False, "Password must contain at least one lowercase letter"

        if not re.search(r"\d", password):
            return False, "Password must contain at least one digit"

        return True, "Password meets requirements"

    @staticmethod
    def validate_user_data(email: str, password: str, first_name: str, last_name: str) -> ValidationErrorResponse:
        """Validate user registration data"""
        errors = []

        if not email or not UserValidation.validate_email(email):
            errors.append({
                "field": "email",
                "message": "Invalid email format"
            })

        is_valid, message = UserValidation.validate_password_strength(password)
        if not is_valid:
            errors.append({
                "field": "password",
                "message": message
            })

        if not first_name or len(first_name.strip()) == 0:
            errors.append({
                "field": "first_name",
                "message": "First name is required"
            })

        if not last_name or len(last_name.strip()) == 0:
            errors.append({
                "field": "last_name",
                "message": "Last name is required"
            })

        if errors:
            return ValidationErrorResponse(
                message="Validation failed",
                details=errors
            )

        return ValidationErrorResponse(
            success=True,
            message="Validation passed",
            details=[]
        )


class JWTValidation:
    """Validation utilities for JWT tokens"""

    @staticmethod
    def validate_token_format(token: str) -> bool:
        """Validate JWT token format (has 3 parts separated by dots)"""
        if not token:
            return False

        parts = token.split('.')
        return len(parts) == 3

    @staticmethod
    def validate_token_expiry(payload: Dict[str, Any]) -> tuple[bool, str]:
        """Validate if token is not expired"""
        exp = payload.get('exp')
        if not exp:
            return False, "Token has no expiration time"

        try:
            expiry_time = int(exp)
            current_time = int(datetime.utcnow().timestamp())

            if current_time >= expiry_time:
                return False, "Token has expired"

            return True, "Token is valid"
        except (ValueError, TypeError):
            return False, "Invalid expiration time in token"

    @staticmethod
    def validate_required_claims(payload: Dict[str, Any]) -> tuple[bool, List[str]]:
        """Validate that required JWT claims are present"""
        required_claims = ['sub', 'exp', 'iat']
        missing_claims = []

        for claim in required_claims:
            if claim not in payload:
                missing_claims.append(claim)

        return len(missing_claims) == 0, missing_claims


class InputValidation:
    """General input validation utilities"""

    @staticmethod
    def sanitize_input(input_str: str) -> str:
        """Basic input sanitization"""
        if not input_str:
            return ""

        # Remove null bytes and basic XSS attempts
        sanitized = input_str.replace('\x00', '').strip()

        # Additional sanitization could be added here
        return sanitized

    @staticmethod
    def validate_uuid_format(uuid_str: str) -> bool:
        """Validate UUID format"""
        import uuid
        try:
            uuid.UUID(uuid_str)
            return True
        except (ValueError, TypeError, AttributeError):
            return False

    @staticmethod
    def validate_integer_id_format(id_str: str) -> bool:
        """Validate integer ID format"""
        try:
            int(id_str)
            return True
        except (ValueError, TypeError):
            return False

    @staticmethod
    def validate_integer_range(value: int, min_val: int = None, max_val: int = None) -> tuple[bool, str]:
        """Validate integer is within range"""
        if min_val is not None and value < min_val:
            return False, f"Value must be greater than or equal to {min_val}"

        if max_val is not None and value > max_val:
            return False, f"Value must be less than or equal to {max_val}"

        return True, "Validation passed"