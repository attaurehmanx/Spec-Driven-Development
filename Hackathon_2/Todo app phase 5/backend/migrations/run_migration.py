"""
Database migration runner script
Applies SQL migrations to the database
"""
import sys
import os
from pathlib import Path

# Add backend directory to path
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

from sqlalchemy import text
from database.session import engine
from config.settings import get_settings
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run_migration(migration_file: str):
    """
    Run a SQL migration file against the database

    Args:
        migration_file: Path to the SQL migration file
    """
    migration_path = Path(__file__).parent / migration_file

    if not migration_path.exists():
        logger.error(f"Migration file not found: {migration_path}")
        return False

    logger.info(f"Reading migration file: {migration_path}")

    try:
        with open(migration_path, 'r') as f:
            sql_content = f.read()

        logger.info("Connecting to database...")
        settings = get_settings()
        logger.info(f"Database URL: {settings.DATABASE_URL.split('@')[1] if '@' in settings.DATABASE_URL else 'configured'}")

        with engine.connect() as connection:
            logger.info("Executing migration...")

            # Split by semicolons and execute each statement
            statements = [s.strip() for s in sql_content.split(';') if s.strip() and not s.strip().startswith('--')]

            for i, statement in enumerate(statements, 1):
                if statement:
                    logger.info(f"Executing statement {i}/{len(statements)}...")
                    connection.execute(text(statement))

            connection.commit()
            logger.info("✓ Migration completed successfully!")
            return True

    except Exception as e:
        logger.error(f"✗ Migration failed: {str(e)}")
        logger.exception(e)
        return False


def main():
    """Main entry point"""
    if len(sys.argv) > 1:
        migration_file = sys.argv[1]
    else:
        # Default to the first migration
        migration_file = "001_add_task_columns.sql"

    logger.info(f"Starting migration: {migration_file}")
    success = run_migration(migration_file)

    if success:
        logger.info("Migration completed successfully!")
        sys.exit(0)
    else:
        logger.error("Migration failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
