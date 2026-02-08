# Use Python 3.11 for better performance and security
FROM python:3.11-slim

# Required Environment Variables (must be provided at runtime):
# - DATABASE_URL: PostgreSQL connection string (e.g., postgresql://user:pass@host/db?sslmode=require)
# - SECRET_KEY: Secret key for JWT token signing/verification
#
# Optional Environment Variables:
# - GEMINI_API_KEY: Google Gemini API key for AI agent service
# - GEMINI_MODEL: Gemini model name (default: gemini-2.5-flash)
# - GEMINI_BASE_URL: Gemini API base URL
# - ALGORITHM: JWT algorithm (default: HS256)
# - ACCESS_TOKEN_EXPIRE_MINUTES: Token expiration time (default: 5)

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Create non-root user for security
RUN useradd -m -u 1000 user && \
    mkdir -p /app && \
    chown -R user:user /app

# Switch to non-root user
USER user
ENV PATH="/home/user/.local/bin:$PATH"

# Set working directory
WORKDIR /app

# Copy and install dependencies first (for better caching)
COPY --chown=user ./requirements.txt requirements.txt
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY --chown=user . /app

# Expose port 8000 (FastAPI default)
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import requests; requests.get('http://localhost:8000/health', timeout=5)" || exit 1

# Run the application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "1"]