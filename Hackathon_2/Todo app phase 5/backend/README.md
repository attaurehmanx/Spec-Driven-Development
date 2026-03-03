# Task Management API with AI Assistant

A production-ready FastAPI backend for a multi-user task management application with AI-powered chat assistance.

## 🚀 Features

- **RESTful API**: Complete CRUD operations for tasks
- **User Authentication**: JWT-based authentication with Better Auth integration
- **AI Chat Assistant**: Google Gemini-powered conversational interface for task management
- **Multi-User Support**: Secure user isolation and data privacy
- **PostgreSQL Database**: Persistent storage with Neon Serverless PostgreSQL
- **Real-time Chat**: Conversation history and context-aware responses

## 📋 API Endpoints

### Authentication
- `POST /auth/signup` - Create new user account
- `POST /auth/signin` - User login with JWT token

### Tasks
- `GET /api/{user_id}/tasks` - List all user tasks
- `POST /api/{user_id}/tasks` - Create new task
- `GET /api/{user_id}/tasks/{id}` - Get task details
- `PUT /api/{user_id}/tasks/{id}` - Update task
- `DELETE /api/{user_id}/tasks/{id}` - Delete task
- `PATCH /api/{user_id}/tasks/{id}/complete` - Toggle task completion

### Chat
- `POST /api/{user_id}/chat` - Send message to AI assistant
- `GET /api/{user_id}/conversations` - List user conversations
- `GET /api/{user_id}/conversations/{id}` - Get conversation history

### Health
- `GET /health` - Service health check
- `GET /docs` - Interactive API documentation (Swagger UI)

## 🔧 Technology Stack

| Component | Technology |
|-----------|-----------|
| Framework | FastAPI |
| Database | Neon PostgreSQL |
| ORM | SQLModel |
| Authentication | JWT + Better Auth |
| AI/LLM | Google Gemini (via OpenAI SDK) |
| Server | Uvicorn |

## 🔐 Security Features

- JWT token-based authentication
- Password hashing with bcrypt
- User data isolation
- CORS protection
- SQL injection prevention via ORM
- Environment-based secrets management

## 📊 Database Schema

### Users
- `id` (UUID, Primary Key)
- `email` (Unique)
- `name`
- `hashed_password`
- `created_at`

### Tasks
- `id` (Integer, Primary Key)
- `user_id` (UUID, Foreign Key)
- `title`
- `description`
- `completed` (Boolean)
- `created_at`
- `updated_at`

### Conversations & Messages
- Conversation tracking with message history
- User-AI message threading
- Timestamp tracking

## 🚦 Getting Started

### Prerequisites

Set the following environment variables in your Hugging Face Space settings:

```bash
DATABASE_URL=postgresql://user:password@host/database
JWT_SECRET_KEY=your-secret-key-min-32-chars
BETTER_AUTH_SECRET=your-auth-secret
GEMINI_API_KEY=your-gemini-api-key
```

### Testing the API

1. **Health Check**
```bash
curl https://YOUR_SPACE.hf.space/health
```

2. **API Documentation**
Visit: `https://YOUR_SPACE.hf.space/docs`

3. **Create Account**
```bash
curl -X POST https://YOUR_SPACE.hf.space/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"user@example.com","password":"SecurePass123!","name":"John Doe"}'
```

4. **Sign In**
```bash
curl -X POST https://YOUR_SPACE.hf.space/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email":"user@example.com","password":"SecurePass123!"}'
```

## 🤖 AI Chat Assistant

The AI assistant can help users:
- Create tasks from natural language
- List and filter tasks
- Update task details
- Mark tasks as complete
- Delete tasks
- Answer questions about tasks

Example conversation:
```
User: "Create a task to buy groceries tomorrow"
AI: "I've created a task 'Buy groceries' with due date tomorrow."

User: "What tasks do I have?"
AI: "You have 3 tasks: 1) Buy groceries (incomplete), 2) Finish report (complete), 3) Call dentist (incomplete)"
```

## 📈 Performance

- **Response Time**: < 200ms for CRUD operations
- **AI Response**: 1-3 seconds (depends on Gemini API)
- **Concurrent Users**: Supports multiple simultaneous users
- **Database**: Serverless PostgreSQL with automatic scaling

## 🔄 Integration

This backend is designed to work with:
- **Frontend**: Next.js 16+ with App Router
- **Auth**: Better Auth (JavaScript/TypeScript)
- **Deployment**: Hugging Face Spaces (Docker)

### CORS Configuration

The API allows requests from:
- `http://localhost:3000` (local development)
- Your production frontend domain (configure in `main.py`)

## 📝 API Response Format

### Success Response
```json
{
  "id": 1,
  "user_id": "uuid-here",
  "title": "Task title",
  "description": "Task description",
  "completed": false,
  "created_at": "2024-01-31T12:00:00",
  "updated_at": "2024-01-31T12:00:00"
}
```

### Error Response
```json
{
  "detail": "Error message here"
}
```

## 🐛 Troubleshooting

### Common Issues

1. **401 Unauthorized**: Check JWT token in Authorization header
2. **404 Not Found**: Verify user_id matches authenticated user
3. **500 Internal Server Error**: Check database connection and environment variables

### Logs

View application logs in the Hugging Face Spaces dashboard under the "Logs" tab.

## 📚 Documentation

- [Full Deployment Guide](./DEPLOYMENT.md)
- [API Documentation](https://YOUR_SPACE.hf.space/docs)
- [Environment Variables](./.env.example)

## 🤝 Contributing

This is a production deployment. For development:
1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt`
3. Set up environment variables
4. Run: `uvicorn main:app --reload`

## 📄 License

Apache 2.0

## 🔗 Links

- [Frontend Repository](#) - Next.js application
- [Neon Database](https://neon.tech) - PostgreSQL hosting
- [Google Gemini](https://ai.google.dev) - AI/LLM provider

---

**Built with**: FastAPI • SQLModel • PostgreSQL • Google Gemini • Better Auth

**Deployed on**: Hugging Face Spaces 🤗
