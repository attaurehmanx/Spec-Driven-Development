-- Migration: Add missing columns to task table
-- Date: 2026-02-16
-- Description: Adds priority, tags, due_date, and recurring columns to support enhanced task features

-- Add priority column with default value 'medium'
ALTER TABLE task
ADD COLUMN IF NOT EXISTS priority VARCHAR(10) NOT NULL DEFAULT 'medium';

-- Add check constraint for priority values
ALTER TABLE task
ADD CONSTRAINT IF NOT EXISTS chk_priority_values
CHECK (priority IN ('high', 'medium', 'low'));

-- Add tags column as PostgreSQL array
ALTER TABLE task
ADD COLUMN IF NOT EXISTS tags TEXT[] NOT NULL DEFAULT '{}';

-- Add due_date column (nullable)
ALTER TABLE task
ADD COLUMN IF NOT EXISTS due_date TIMESTAMP;

-- Add recurring column with default value 'none'
ALTER TABLE task
ADD COLUMN IF NOT EXISTS recurring VARCHAR(10) NOT NULL DEFAULT 'none';

-- Add check constraint for recurring values
ALTER TABLE task
ADD CONSTRAINT IF NOT EXISTS chk_recurring_values
CHECK (recurring IN ('none', 'daily', 'weekly', 'monthly'));

-- Create index on priority for faster filtering
CREATE INDEX IF NOT EXISTS idx_task_priority ON task(priority);

-- Create index on due_date for faster sorting and overdue queries
CREATE INDEX IF NOT EXISTS idx_task_due_date ON task(due_date);

-- Create GIN index on tags array for faster tag searches
CREATE INDEX IF NOT EXISTS idx_task_tags ON task USING GIN(tags);
