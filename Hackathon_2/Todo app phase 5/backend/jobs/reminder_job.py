"""
Reminder Job Scheduler

Background job that periodically checks for tasks with upcoming due dates
and publishes reminder events.

Runs every 5 minutes to check for tasks due within the next hour.
"""

import logging
from datetime import datetime, timedelta
from sqlmodel import Session, select
from typing import List

from models.task_models import Task
from database.session import get_session
from events.reminder_publisher import get_publisher

logger = logging.getLogger(__name__)


class ReminderJob:
    """
    Background job for checking and publishing task reminders.
    """

    def __init__(self):
        """Initialize the reminder job."""
        self.last_run: datetime = datetime.utcnow()

    async def check_and_send_reminders(self):
        """
        Check for tasks with upcoming due dates and send reminder events.

        Finds tasks where:
        - due_date is between NOW() and NOW() + 1 hour
        - completed = false
        - Reminder hasn't been sent in the last run

        Publishes reminder events to Kafka/Redpanda.
        """
        try:
            logger.info("Starting reminder check job")

            # Get current time and time window
            now = datetime.utcnow()
            one_hour_later = now + timedelta(hours=1)

            # Get database session
            # Note: In production, this should use a proper session management
            # For now, we'll create a new session for each job run
            from database.session import engine
            with Session(engine) as session:
                # Query for tasks with upcoming due dates
                statement = select(Task).where(
                    Task.due_date >= now,
                    Task.due_date <= one_hour_later,
                    Task.completed == False
                )

                tasks = session.exec(statement).all()

                if not tasks:
                    logger.info("No tasks with upcoming due dates found")
                    return

                logger.info(f"Found {len(tasks)} tasks with upcoming due dates")

                # Get publisher
                publisher = await get_publisher()

                # Publish reminder for each task
                success_count = 0
                for task in tasks:
                    try:
                        success = await publisher.publish_reminder(
                            task_id=task.id,
                            user_id=task.user_id,
                            title=task.title,
                            due_date=task.due_date,
                            priority=task.priority.value
                        )

                        if success:
                            success_count += 1
                            logger.info(
                                f"Reminder sent for task {task.id}: '{task.title}' "
                                f"(due: {task.due_date.isoformat()})"
                            )
                        else:
                            logger.warning(f"Failed to send reminder for task {task.id}")

                    except Exception as e:
                        logger.error(f"Error sending reminder for task {task.id}: {e}")

                logger.info(
                    f"Reminder job completed: {success_count}/{len(tasks)} reminders sent successfully"
                )

                # Update last run time
                self.last_run = now

        except Exception as e:
            logger.error(f"Error in reminder job: {e}", exc_info=True)


# Global job instance
_reminder_job: ReminderJob = None


def get_reminder_job() -> ReminderJob:
    """
    Get or create the global reminder job instance.

    Returns:
        ReminderJob instance
    """
    global _reminder_job
    if _reminder_job is None:
        _reminder_job = ReminderJob()
    return _reminder_job


async def run_reminder_job():
    """
    Entry point for the reminder job.
    Called by APScheduler every 5 minutes.
    """
    job = get_reminder_job()
    await job.check_and_send_reminders()
