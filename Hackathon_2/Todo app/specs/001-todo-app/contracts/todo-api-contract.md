# Todo API Contract

## Overview
This document describes the contract for the Todo Console Application operations. Since this is a console application, these represent the internal service contracts that the CLI layer will interact with.

## Todo Entity
```
{
  "id": integer,           // Unique incremental identifier
  "title": string,         // Required title (non-empty)
  "description": string,   // Optional description
  "status": "Complete" | "Incomplete"  // Completion status
}
```

## Operations

### 1. Add Todo
- **Method**: POST (via service call)
- **Input**: { title: string, description?: string }
- **Output**: Todo object with assigned ID and "Incomplete" status
- **Success**: Returns the created Todo object
- **Errors**:
  - ValidationError: If title is empty

### 2. Get All Todos
- **Method**: GET (via service call)
- **Input**: None
- **Output**: Array of Todo objects
- **Success**: Returns all todos in the system
- **Errors**: None

### 3. Get Todo by ID
- **Method**: GET (via service call)
- **Input**: { id: integer }
- **Output**: Todo object
- **Success**: Returns the requested Todo object
- **Errors**:
  - NotFoundError: If todo with given ID doesn't exist

### 4. Update Todo
- **Method**: PUT (via service call)
- **Input**: { id: integer, title?: string, description?: string }
- **Output**: Updated Todo object
- **Success**: Returns the updated Todo object
- **Errors**:
  - NotFoundError: If todo with given ID doesn't exist
  - ValidationError: If title is empty

### 5. Delete Todo
- **Method**: DELETE (via service call)
- **Input**: { id: integer }
- **Output**: boolean (success indicator)
- **Success**: Returns true if deletion was successful
- **Errors**:
  - NotFoundError: If todo with given ID doesn't exist

### 6. Toggle Todo Status
- **Method**: PATCH (via service call)
- **Input**: { id: integer }
- **Output**: Updated Todo object
- **Success**: Returns the Todo object with toggled status
- **Errors**:
  - NotFoundError: If todo with given ID doesn't exist

## Error Format
```
{
  "error": string,
  "code": string,
  "details": object (optional)
}
```

## Validation Rules
- All IDs must be positive integers
- Title is required and must not be empty
- Description is optional
- Status must be either "Complete" or "Incomplete"