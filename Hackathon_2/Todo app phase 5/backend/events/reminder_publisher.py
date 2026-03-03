"""
Reminder Event Publisher

Publishes reminder events to Kafka/Redpanda for tasks approaching their due dates.
This is a temporary implementation using aiokafka directly.
Will be refactored to use Dapr HTTP API in Phase 5 (T071-T074).
"""

import json
import logging
from typing import Dict, Any, Optional
from datetime import datetime
from aiokafka import AIOKafkaProducer
from aiokafka.errors import KafkaError

logger = logging.getLogger(__name__)


class ReminderPublisher:
    """
    Publishes task reminder events to Kafka/Redpanda.

    Temporary implementation using aiokafka directly.
    Will be replaced with Dapr HTTP API in Phase 5.
    """

    def __init__(self, bootstrap_servers: str = "localhost:9092"):
        """
        Initialize the reminder publisher.

        Args:
            bootstrap_servers: Kafka bootstrap servers (default: localhost:9092)
        """
        self.bootstrap_servers = bootstrap_servers
        self.producer: Optional[AIOKafkaProducer] = None
        self.topic = "tasks.reminder"

    async def start(self):
        """Start the Kafka producer."""
        try:
            self.producer = AIOKafkaProducer(
                bootstrap_servers=self.bootstrap_servers,
                value_serializer=lambda v: json.dumps(v).encode('utf-8'),
                key_serializer=lambda k: k.encode('utf-8') if k else None
            )
            await self.producer.start()
            logger.info(f"Reminder publisher started, connected to {self.bootstrap_servers}")
        except KafkaError as e:
            logger.error(f"Failed to start reminder publisher: {e}")
            raise

    async def stop(self):
        """Stop the Kafka producer."""
        if self.producer:
            await self.producer.stop()
            logger.info("Reminder publisher stopped")

    async def publish_reminder(
        self,
        task_id: int,
        user_id: str,
        title: str,
        due_date: datetime,
        priority: str = "medium"
    ) -> bool:
        """
        Publish a reminder event for a task.

        Args:
            task_id: Task ID
            user_id: User ID who owns the task
            title: Task title
            due_date: Task due date
            priority: Task priority level

        Returns:
            True if published successfully, False otherwise
        """
        if not self.producer:
            logger.error("Producer not started, cannot publish reminder")
            return False

        try:
            # Create reminder event payload
            event = {
                "task_id": task_id,
                "user_id": user_id,
                "title": title,
                "due_date": due_date.isoformat(),
                "priority": priority,
                "event_type": "task_reminder",
                "timestamp": datetime.utcnow().isoformat()
            }

            # Publish to Kafka topic
            # Use task_id as key for partitioning
            await self.producer.send_and_wait(
                self.topic,
                value=event,
                key=str(task_id)
            )

            logger.info(f"Published reminder for task {task_id} (user: {user_id})")
            return True

        except KafkaError as e:
            logger.error(f"Failed to publish reminder for task {task_id}: {e}")
            return False
        except Exception as e:
            logger.error(f"Unexpected error publishing reminder for task {task_id}: {e}")
            return False


# Global publisher instance
_publisher: Optional[ReminderPublisher] = None


async def get_publisher() -> ReminderPublisher:
    """
    Get or create the global reminder publisher instance.

    Returns:
        ReminderPublisher instance
    """
    global _publisher
    if _publisher is None:
        # Get Kafka bootstrap servers from environment or use default
        import os
        bootstrap_servers = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "localhost:9092")
        _publisher = ReminderPublisher(bootstrap_servers)
        await _publisher.start()
    return _publisher


async def shutdown_publisher():
    """Shutdown the global reminder publisher."""
    global _publisher
    if _publisher:
        await _publisher.stop()
        _publisher = None
