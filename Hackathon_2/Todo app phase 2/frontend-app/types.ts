// Task-related types
export interface Task {
  id: string; // UUID
  title: string;
  description?: string;
  completed: boolean;
  owner_id: string; // UUID
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
}

export interface TaskFormData {
  title: string;
  description?: string;
  completed: boolean;
}

export interface TaskListResponse {
  tasks: Task[];
}

export interface TaskResponse {
  id: string; // UUID
  title: string;
  description?: string;
  completed: boolean;
  owner_id: string; // UUID
  created_at: string; // ISO date string
  updated_at: string; // ISO date string
}

export interface ErrorResponse {
  error: string;
  message?: string;
}

// Auth-related types
export interface UserSession {
  id: string;
  email: string;
  name?: string;
  first_name?: string;
  last_name?: string;
  createdAt: string;
  updatedAt?: string;
  phone?: string;
  address?: string;
  avatar?: string;
}

export interface AuthUIState {
  isAuthenticated: boolean;
  user: UserSession | null;
  loading: boolean;
  error: string | null;
}