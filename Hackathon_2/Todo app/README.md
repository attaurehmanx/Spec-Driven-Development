# Todo Console Application

A simple todo console application built with Python that allows users to manage tasks through a command-line interface.

## Features

- Add new todos with title and optional description
- View all todos with ID, title, description, and status
- Update existing todos
- Delete todos (with confirmation)
- Mark todos as complete/incomplete

## Requirements

- Python 3.13 or higher
- UV package manager

## Setup

1. Clone the repository
2. Navigate to project directory
3. Install dependencies with UV:
   ```bash
   uv sync
   ```

## Usage

Run the console application:
```bash
python src/cli/main.py
```

## Project Structure

```
src/
├── models/
│   └── todo.py          # Todo data model
├── services/
│   └── todo_service.py  # Todo business logic
├── cli/
│   └── main.py          # Console interface
└── lib/
    └── storage.py       # In-memory storage implementation
```