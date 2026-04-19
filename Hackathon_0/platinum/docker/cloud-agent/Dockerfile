# Platinum Cloud Agent Dockerfile
# Based on Python 3.12

FROM python:3.12-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements from parent directory
COPY ../requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code from parent directory
COPY ../cloud/ ./cloud/
COPY ../shared/ ./shared/

# Create logs directory
RUN mkdir -p /app/logs

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Health check
HEALTHCHECK --interval=60s --timeout=10s --start-period=30s --retries=3 \
    CMD python -c "import os; print('Cloud Agent healthy')" || exit 1

# Run Cloud Agent
CMD ["python", "cloud/agents/cloud_agent_fixed.py"]
